#!/usr/bin/env python3
"""Offline, diagnostic cylinder-axis fitting from one saved SAM3 COCO-RLE mask.

Never calls SAM3 and never changes RGB, depth, CAD, or saved SAM3 results.
The fitted line is in the aligned-RGB camera frame, in millimetres.  It is not
a CAD 6D pose, a mass centre, or a robot-approved grasp target.

Typical use (after putting *verified* matching paths/intrinsics in the config):
  C:/Users/14817/miniconda3/envs/pt/python.exe sam3/test/fit_bottle_axis.py --instance-id 1 --show-3d

Run ``--synthetic-test`` first.  ``--verify-source`` reconstructs the historic
SAM3 visualization against candidate RGBs and is deliberately read-only.
"""
import argparse, json, math, os, sys, tempfile, subprocess, time, threading
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from pycocotools import mask as mask_utils
from scipy.optimize import least_squares

# Candidate speed knobs (defaults preserve the original serial/numeric behaviour):
_HERE = str(Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)  # spawn-pool workers must be able to import this module by name
_USE_ANALYTIC_JAC = os.environ.get('FIT_ANALYTIC_JAC', '0').strip().lower() in ('1', 'true', 'yes', 'on')
_BOOTSTRAP_WORKERS_DEFAULT = max(1, int(os.environ.get('FIT_BOOTSTRAP_WORKERS', '1') or 1))
_MAIN_FIT_POOL = os.environ.get('FIT_MAIN_FIT_POOL', '0').strip().lower() in ('1', 'true', 'yes', 'on')
# Bootstrap replicate count (stability audit).  Default 12 preserves the historical gate;
# a lower value (e.g. 4) relaxes the audit: same seeded sample sequence, max over fewer
# replicates can only move the gate in the looser direction.  Thresholds are unchanged.
_BOOTSTRAP_BOOTS_DEFAULT = max(1, int(os.environ.get('FIT_BOOTSTRAP_BOOTS', '12') or 12))

# =========================== editable absolute defaults ====================
PROJECT = Path(r"D:\TermiTech\Cosmetics_Sort")
DEFAULT_RESULT_JSON = PROJECT / r"sam3\test\results\20260915_152918\head_rgb.json"
# Verified by pixel-exact reconstruction of the saved SAM3 visualization.
# This metadata file does NOT contain intrinsics, so a default real run remains
# intentionally rejected until it is replaced with the source camera-intrinsic JSON.
DEFAULT_RGB_PATH = PROJECT / r"data\20260915\120045958\head_rgb.jpg"
DEFAULT_DEPTH_PATH = PROJECT / r"data\20260915\120045958\head_depth_aligned.npy"
DEFAULT_CAMERA_PATH = PROJECT / r"data\20260915\120045958\camera.json"
DEFAULT_CAD_PATH = PROJECT / r"foundationpose\data\1\CAD\Avene\Avene-1\Avene.obj.obj"
DEFAULT_INSTANCE_ID = 1                    # 1-based detection number
DEFAULT_OUTPUT_ROOT = PROJECT / r"sam3\test\axis_fit_results"
DEFAULT_DEPTH_SCALE = 1.0                  # source depth is documented mm
DEFAULT_BODY_RADIUS_MM = 28.5               # confirmed bottle diameter 57 mm / 2
DEFAULT_AXIS_PRIOR = None                   # e.g. "0,1,0", never assume camera axes
DEFAULT_AXIS_PRIOR_FRAME = "camera"        # only camera prior is accepted by fitter
SUPPORT_ROOT = PROJECT / r"sam3\test\axis_fit_support\head_camera_transform_20260915"
DEFAULT_EXTRINSICS_PATH = SUPPORT_ROOT / "transforms_20260915.json"
DEFAULT_HANDEYE_PATH = SUPPORT_ROOT / r"calibration\head_camera_handeye_20260914.json"
DEFAULT_SAMPLE_ID = None                      # defaults to DEFAULT_RGB_PATH.parent.name
DEFAULT_Z_REF_MM = 1200.0                     # uniform diagnostic chassis plane; NOT measured ground/box/grasp height
DEFAULT_AXIS_PRIOR_MODE = "chassis_up"       # chassis_up | manual | none
DEFAULT_FIT_MODE = "prior_2d"                 # prior_2d (default) | cylinder_3d; never silently switches
MAX_BOOTSTRAP_VISIBLE_CENTER_MM = 8.0          # retained existing acceptance threshold
MAX_BOOTSTRAP_REFERENCE_POINT_MM = 10.0        # new fixed-plane stability gate
MAX_BOOTSTRAP_AXIS_ANGLE_DEG = 6.0             # cylinder_3d only; prior_2d direction is hard constrained
MIN_BODY_FIT_POINTS = 1000                     # reject small bottle-head-only masks
MIN_VISIBLE_AXIS_SPAN_MM = 40.0                # reject insufficient axial support
DEFAULT_SHOW_3D = False
DEFAULT_MASK_KIND = "whole_bottle"            # bottle_body keeps all eroded body-mask rows; no 12% top/bottom trim
DEFAULT_MASK_EROSION_PIXELS = 4
DEFAULT_BODY_MASK_COMPARISON_ROOT = PROJECT / r"sam3\test\axis_fit_results\body_mask_comparison"
DEFAULT_BODY_RESULT_A = PROJECT / r"sam3\test\results\20260915_181634\head_rgb.json"
DEFAULT_BODY_RESULT_B = PROJECT / r"sam3\test\results\20260915_181724\head_rgb.json"
DEFAULT_BODY_Z_MM = None                    # legacy option; use DEFAULT_Z_REF_MM/chassis plane
DEFAULT_BODY_HEIGHT_MM = None               # support_relative mode; no invented value
DEFAULT_T_ROBOT_CAMERA_PATH = None          # verified 4x4 JSON only
DEFAULT_SUPPORT_PLANE_PATH = None           # verified JSON: point_mm, normal, frame
# ============================================================================

def norm(v):
    v=np.asarray(v,dtype=float); n=np.linalg.norm(v)
    if n == 0: raise ValueError("zero direction")
    return v/n
def jdump(x): return json.dumps(x, ensure_ascii=False, indent=2, default=lambda o: o.tolist() if isinstance(o,np.ndarray) else str(o))
def rle(d):
    c=d["segmentation"]["counts"]; c=c.encode() if isinstance(c,str) else c
    m=mask_utils.decode({"size":d["segmentation"]["size"],"counts":c})
    return (m[...,0] if m.ndim==3 else m).astype(bool)
def parse_vec(s): return None if s is None else norm([float(x) for x in s.split(",")])
def read_obj_extent(path):
    vs=[]
    with Path(path).open(encoding="utf-8",errors="replace") as f:
        for line in f:
            if line.startswith("v "):
                try: vs.append([float(x) for x in line.split()[1:4]])
                except ValueError: pass
    if not vs: raise ValueError("OBJ contains no vertex records")
    v=np.asarray(vs); lo=v.min(0); hi=v.max(0)
    return {"path":str(Path(path).resolve()),"vertex_count":len(v),"min":lo,"max":hi,"extent":hi-lo,
            "longest_mesh_axis_index":int(np.argmax(hi-lo)),
            "unit":"UNKNOWN (OBJ has no reliable unit declaration)",
            "radius_mm":None,"radius_source":"not inferred from whole-mesh bounding box"}

def camera_from_json(path):
    """Read common, explicit aligned-colour pinhole schemas; reject metadata-only JSON."""
    d=json.loads(Path(path).read_text(encoding="utf-8")); candidates=[d,d.get("intrinsics",{}),d.get("color_intrinsics",{}),d.get("camera",{})]
    for c in candidates:
        if all(k in c for k in ("fx","fy","cx","cy")):
            return {k:float(c[k]) for k in ("fx","fy","cx","cy")}, d
        if "camera_matrix" in c:
            a=np.asarray(c["camera_matrix"],float).reshape(3,3); return {"fx":a[0,0],"fy":a[1,1],"cx":a[0,2],"cy":a[1,2]},d
        # Saved head-camera schema: row-major 3x3 RGB pinhole matrix.
        if "cam_K" in c:
            a=np.asarray(c["cam_K"],float)
            if a.size != 9:
                raise ValueError("cam_K must contain exactly 9 values")
            a=a.reshape(3,3)
            if a[0,0] <= 0 or a[1,1] <= 0 or not np.allclose(a[2],[0,0,1]):
                raise ValueError("cam_K is not a valid pinhole camera matrix")
            return {"fx":a[0,0],"fy":a[1,1],"cx":a[0,2],"cy":a[1,2]},d
    # Complete hand-eye calibration schema: this is an explicit calibrated-K
    # fallback, not a claim that a missing capture-side camera.json exists.
    c=d.get("intrinsics",{})
    if "camera_matrix" in c:
        a=np.asarray(c["camera_matrix"],float).reshape(3,3)
        if a[0,0]>0 and a[1,1]>0 and np.allclose(a[2],[0,0,1]):
            return {"fx":a[0,0],"fy":a[1,1],"cx":a[0,2],"cy":a[1,2]},d
    raise ValueError("camera JSON has no explicit fx/fy/cx/cy or camera_matrix; recording metadata alone is insufficient")

def backproject(mask, depth_mm, K):
    ys,xs=np.nonzero(mask); z=depth_mm[ys,xs]; good=np.isfinite(z)&(z>0)
    ys,xs,z=ys[good],xs[good],z[good]
    p=np.column_stack(((xs-K['cx'])*z/K['fx'],(ys-K['cy'])*z/K['fy'],z))
    return p,xs,ys
def eroded(mask, pixels=4):
    return cv2.erode(mask.astype(np.uint8),np.ones((2*pixels+1,2*pixels+1),np.uint8)).astype(bool)
def body_filter(mask, depth, erosion=4, mask_kind="whole_bottle"):
    """Conservative image-domain filter. It exposes rather than hides rejected pixels."""
    inner=eroded(mask,erosion); ys,xs=np.nonzero(mask)
    if len(ys)==0:return inner
    if mask_kind not in ("whole_bottle","bottle_body"): raise ValueError("mask_kind must be whole_bottle or bottle_body")
    # Whole-bottle masks need a conservative shoulder/cap/contact trim.  A
    # bottle_body mask already asserts the straight body: do NOT secretly trim
    # its top/bottom rows merely to lower residuals.
    if mask_kind=="whole_bottle":
        y0,y1=np.percentile(ys,[12,88]); body=inner&(np.indices(mask.shape)[0]>=y0)&(np.indices(mask.shape)[0]<=y1)
    else: body=inner.copy()
    vals=depth[body & np.isfinite(depth) & (depth>0)]
    if len(vals)>20:
        lo,hi=np.percentile(vals,[2,98]); body &= (depth>=lo)&(depth<=hi)
    return body

def save_mask_regions(rgb, original, eroded_mask, fit_mask, path, mask_kind):
    """Audit image: original=yellow, erosion-loss=red, fit-base=green."""
    out=rgb.copy(); out[original]=(0.55*out[original]+.45*np.array([0,190,255])).astype(np.uint8)
    removed=original&~eroded_mask; out[removed]=(0.25*out[removed]+.75*np.array([0,0,255])).astype(np.uint8)
    out[fit_mask]=(0.30*out[fit_mask]+.70*np.array([0,220,0])).astype(np.uint8)
    text='mask-kind={} | yellow=original red=erosion-loss green=fit-base'.format(mask_kind)
    cv2.putText(out,text,(12,28),cv2.FONT_HERSHEY_SIMPLEX,.58,(0,0,0),3,cv2.LINE_AA);cv2.putText(out,text,(12,28),cv2.FONT_HERSHEY_SIMPLEX,.58,(255,255,255),1,cv2.LINE_AA)
    cv2.imwrite(str(path),out)
def basis(axis):
    a=norm(axis); seed=np.array([1.,0,0]) if abs(a[0])<.8 else np.array([0.,1,0]); u=norm(np.cross(a,seed)); return u,norm(np.cross(a,u))
def fit_prior(points,radius,axis):
    """Known-axis circle fit with deliberately diverse, reproducible starts.

    A partial visible cylinder is ambiguous enough that a median-only start can
    settle on a worse local solution.  The returned diagnostics retain all
    distinct candidate centres so this ambiguity is auditable rather than
    silently absorbed into a bootstrap number.
    """
    a=norm(axis); u,v=basis(a); q=np.c_[points@u,points@v]
    def fun(c): return np.sqrt(((q-c)**2).sum(1))-radius
    centre=np.median(q,0)
    # median/mean plus one radius around the projected cross-section.  These
    # starts are geometry-scaled, not arbitrary scene coordinates.
    starts=[centre, np.mean(q,0)]
    starts += [centre+radius*np.array([math.cos(t),math.sin(t)]) for t in np.linspace(0,2*np.pi,12,endpoint=False)]
    candidates=[]; solve_wall_ms=0.0; solve_nfev=0; solve_calls=0
    for start in starts:
        try:
            t_solve=time.perf_counter()
            sol=least_squares(fun,start,loss='soft_l1',f_scale=2.0,max_nfev=1500)
            solve_wall_ms += (time.perf_counter()-t_solve)*1000.0; solve_nfev += int(getattr(sol,'nfev',0)); solve_calls += 1
            if sol.success and np.all(np.isfinite(sol.x)) and np.isfinite(sol.cost):
                candidates.append((sol, np.asarray(start,float)))
        except Exception:
            pass
    if not candidates: raise ValueError("prior_2d circle optimizer produced no successful finite candidate")
    candidates.sort(key=lambda item: item[0].cost)
    sol,start=candidates[0]
    # Solutions within 2% robust cost are considered near-equivalent.  Only
    # spatially distinct centres count as ambiguity; repeated starts converging
    # to the same answer do not.
    near=[item for item in candidates if item[0].cost <= sol.cost*1.02+1e-9]
    unique=[]
    for item in near:
        if all(np.linalg.norm(item[0].x-old[0].x)>2.0 for old in unique): unique.append(item)
    spread=max([float(np.linalg.norm(x[0].x-sol.x)) for x in unique],default=0.0)
    p0=sol.x[0]*u+sol.x[1]*v # canonical p0: perpendicular to a
    summaries=[{"initial_center_plane_mm":s,"final_center_plane_mm":x.x,"optim_cost":float(x.cost),"median_abs_radial_mm":float(np.median(np.abs(fun(x.x))))} for x,s in candidates]
    return p0,a,fun(sol.x),{"method":"known-radius circle in plane normal to supplied camera-frame axis; multi-start robust solve","optim_cost":float(sol.cost),"candidate_count":len(candidates),"near_best_candidate_count":len(near),"near_best_unique_center_count":len(unique),"near_best_center_spread_mm":spread,"candidate_solution_ambiguity":bool(len(unique)>1 and spread>2.0),"least_squares_calls":solve_calls,"least_squares_nfev":solve_nfev,"least_squares_wall_ms":solve_wall_ms,"candidates":summaries}
def axis_from_angles(theta,phi): return np.array([math.cos(theta)*math.sin(phi),math.sin(theta)*math.sin(phi),math.cos(phi)])
def _jac_fit_3d(x,points,radius):
    """Analytic Jacobian of fit_3d's residual, exact within the current basis() seed branch.

    Residual row i: ||(q_i-p) x a|| - radius with a=axis_from_angles(x0,x1), u,v=basis(a),
    p=x2*u+x3*v (p is perpendicular to a by construction).  Chain rule:
      dr_i/dk = (-d.dp_k - m*dm_k)/L,  d=q_i-p, m=d.a, L=||d x a||, dm_k=d.da_k - dp_k.a.
    Verified against central/2-point finite differences by sam3/test jacobian verification
    scripts before being enabled; FIT_ANALYTIC_JAC=0 keeps the original numeric path."""
    a=axis_from_angles(x[0],x[1]); u,v=basis(a); p=x[2]*u+x[3]*v
    st=math.sin(x[0]); ct=math.cos(x[0]); sp=math.sin(x[1]); cp=math.cos(x[1])
    da_dtheta=np.array([-st*sp,ct*sp,0.0]); da_dphi=np.array([ct*cp,st*cp,-sp])
    seed=np.array([1.,0,0]) if abs(a[0])<.8 else np.array([0.,1,0])
    w=np.cross(a,seed); n=np.linalg.norm(w)
    eye=np.eye(3); du=np.empty((3,3)); dv=np.empty((3,3))
    for j in range(3):
        dw=np.cross(eye[j],seed); dn=(w@dw)/n
        duj=dw/n-w*(dn)/(n*n)
        du[:,j]=duj; dv[:,j]=np.cross(eye[j],u)+np.cross(a,duj)
    dp_dtheta=x[2]*(du@da_dtheta)+x[3]*(dv@da_dtheta)
    dp_dphi=x[2]*(du@da_dphi)+x[3]*(dv@da_dphi)
    d=points-p; m=d@a; L=np.linalg.norm(np.cross(d,a),axis=1)
    cols=[]
    for dpk,dak in ((dp_dtheta,da_dtheta),(dp_dphi,da_dphi),(u,None),(v,None)):
        if dak is None: dm=-dpk@a
        else: dm=d@dak-dpk@a
        cols.append((-(d@dpk)-m*dm)/L)
    return np.column_stack(cols)
def fit_3d(points,radius,initial):
    # p0 is constrained perpendicular to axis, avoiding meaningless p0 drift along the same line.
    def unpack(x):
        a=axis_from_angles(x[0],x[1]); u,v=basis(a); return x[2]*u+x[3]*v,a
    def fun(x):
        p,a=unpack(x); return np.linalg.norm(np.cross(points-p,a),axis=1)-radius
    def jac(x):
        return _jac_fit_3d(x,points,radius)
    a=norm(initial); th=math.atan2(a[1],a[0]); ph=math.acos(np.clip(a[2],-1,1)); t_solve=time.perf_counter()
    if _USE_ANALYTIC_JAC:
        sol=least_squares(fun,[th,ph,0,0],jac=jac,loss='soft_l1',f_scale=2.0,max_nfev=1500)
    else:
        sol=least_squares(fun,[th,ph,0,0],loss='soft_l1',f_scale=2.0,max_nfev=1500)
    solve_ms=(time.perf_counter()-t_solve)*1000.0
    p,a=unpack(sol.x); return p,a,fun(sol.x),{"method":"known-radius robust 3D cylinder; PCA/random starts only initialize","optim_cost":float(sol.cost),"least_squares_calls":1,"least_squares_nfev":int(getattr(sol,'nfev',0)),"least_squares_njev":int(getattr(sol,'njev',0)),"jacobian":"analytic" if _USE_ANALYTIC_JAC else "numeric_2point","least_squares_wall_ms":solve_ms}
def canonical(a,up=None):
    a=norm(a)
    if up is not None and np.dot(a,up)<0:a=-a
    elif up is None and tuple(a)<tuple(-a):a=-a
    return a
def line_plane_intersection(p,a,plane_point,plane_normal,eps=1e-8):
    den=float(np.dot(plane_normal,a))
    if abs(den)<eps: raise ValueError("axis is parallel/nearly parallel to reference plane")
    s=float(np.dot(plane_normal,plane_point-p)/den)
    return p+s*a,s

def _cylinder_trial_task(args):
    """Module-level fit_3d trial so the persistent pool can execute main-fit multistart.

    Same function, same inputs and same selection rule as the serial loop; only the
    execution site changes. Workers run the identical code path."""
    x,radius,start=args
    return fit_3d(x,radius,start)

def _pool_worker_child():
    """True inside a bootstrap-pool worker process (initializer marks the env).

    Guards against nested pools: a worker executing _cylinder_base_solver must run its
    own trials serially, never submit them back to a pool."""
    return os.environ.get('_FIT_POOL_WORKER') == '1'

def _cylinder_base_solver(x,radius):
    _,_,vh=np.linalg.svd(x-x.mean(0),full_matrices=False); starts=[vh[0],vh[1],vh[2],np.array([0,1.,0])]
    trials=[];pool_meta=None
    if _MAIN_FIT_POOL and not _pool_worker_child() and _BOOTSTRAP_WORKERS_DEFAULT>1:
        try:
            t0=time.perf_counter()
            pool,created,create_ms=_get_bootstrap_pool(_BOOTSTRAP_WORKERS_DEFAULT)
            t_submit=time.perf_counter()
            futures=[pool.submit(_cylinder_trial_task,(x,radius,s)) for s in starts]
            submit_ms=round((time.perf_counter()-t_submit)*1000.0,3)
            trials=[f.result() for f in futures]
            pool_meta={'method':'process_pool','workers':int(_BOOTSTRAP_WORKERS_DEFAULT),'fallback_serial':False,'pool_error':None,
                       'pool_created_this_call':bool(created),'pool_create_ms':create_ms,'submit_ms':submit_ms,
                       'collect_ms':round((time.perf_counter()-t_submit)*1000.0,3)-submit_ms,'candidate_count_submitted':len(starts)}
        except Exception as exc:
            pool_meta={'method':'process_pool','fallback_serial':True,'pool_error':'{}: {}'.format(type(exc).__name__,exc)}
            trials=[]
    elif _MAIN_FIT_POOL:
        pool_meta={'method':'serial_inprocess','fallback_serial':False,'pool_error':None,
                   'reason':'inside pool worker' if _pool_worker_child() else 'workers<=1'}
    if not trials:
        for s in starts:
            try: trials.append(fit_3d(x,radius,s))
            except Exception: pass
    if not trials: raise ValueError("cylinder_3d optimizer produced no candidate")
    chosen=min(trials,key=lambda z:z[3]["optim_cost"])
    metrics=chosen[3]
    metrics["least_squares_calls"]=sum(int(t[3].get("least_squares_calls",0)) for t in trials)
    metrics["least_squares_nfev"]=sum(int(t[3].get("least_squares_nfev",0)) for t in trials)
    metrics["least_squares_njev"]=sum(int(t[3].get("least_squares_njev",0) or 0) for t in trials)
    metrics["least_squares_wall_ms"]=sum(float(t[3].get("least_squares_wall_ms",0.0)) for t in trials)
    metrics["candidate_count"]=len(trials)
    if pool_meta is not None: metrics["main_fit_pool"]=pool_meta
    return chosen

def _estimate_full(points,radius,prior,mode):
    """The same initial-fit / inlier / refit procedure for main fit and every bootstrap.

    Module-level single source of truth so the process-pool workers execute exactly the
    same numerical path as the serial in-process fit (no per-request state captured)."""
    if mode=="prior_2d":
        if prior is None: raise ValueError("prior_2d requires an axis prior in camera frame")
        def base(x): return fit_prior(x,radius,prior)
    elif mode=="cylinder_3d":
        def base(x): return _cylinder_base_solver(x,radius)
    else: raise ValueError("mode must be cylinder_3d or prior_2d")
    p0,a0,res0,extra0=base(points); a0=canonical(a0,prior)
    prelim=np.abs(res0)<=max(3.0,np.percentile(np.abs(res0),65))
    if prelim.sum()<80: raise ValueError("too few preliminary inliers for refit")
    p1,a1,res1,extra1=base(points[prelim]); a1=canonical(a1,prior)
    res1=np.linalg.norm(np.cross(points-p1,a1),axis=1)-radius
    extra1["initial_fit"]={k:v for k,v in extra0.items() if k!="candidates"}
    return p1,a1,res1,extra1

_BOOTSTRAP_POOL=None
_BOOTSTRAP_POOL_LOCK=threading.Lock()
_BOOTSTRAP_POOL_META={'workers':0,'created_at':None,'create_ms':0.0,'method':'spawn','first_task_spawn_ms':None}

def _bootstrap_worker_init():
    """Runs once per persistent worker: pin BLAS/OpenMP threads so worker count x BLAS threads
    cannot oversubscribe the CPU.  Runs before numpy is imported in the spawn child."""
    for var in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS','VECLIB_MAXIMUM_THREADS'):
        os.environ.setdefault(var,'1')
    os.environ['_FIT_POOL_WORKER']='1'

def _bootstrap_row(sub,radius,prior,mode,a,c0,r0,common_plane,reference_plane):
    """One bootstrap replicate; identical arithmetic to the historical in-loop closure."""
    try:
        pp,aa,_,_=_estimate_full(sub,radius,prior,mode); aa=canonical(aa,prior)
        ci,_=line_plane_intersection(pp,aa,*common_plane)
        row={'angle':float(np.degrees(np.arccos(np.clip(abs(np.dot(a,aa)),-1,1)))),'visible':float(np.linalg.norm(ci-c0)),'fixed':None,'visible_error':None,'reference_error':None}
        if r0 is not None:
            try:
                ri,_=line_plane_intersection(pp,aa,*reference_plane); row['fixed']=float(np.linalg.norm(ri-r0))
            except Exception as exc: row['reference_error']=str(exc)
        return row
    except Exception as exc:
        return {'angle':None,'visible':None,'fixed':None,'visible_error':str(exc),'reference_error':None}

def _bootstrap_task(payload):
    sub,radius,prior,mode,a,c0,r0,common_plane,reference_plane=payload
    t0=time.perf_counter()
    row=_bootstrap_row(sub,radius,prior,mode,a,c0,r0,common_plane,reference_plane)
    row['_worker_compute_ms']=round((time.perf_counter()-t0)*1000.0,3)
    return row

def _get_bootstrap_pool(workers):
    """Persistent module-level process pool (survives across requests); spawn context avoids
    forking from a multi-threaded HTTP server; workers import this module by name."""
    global _BOOTSTRAP_POOL
    with _BOOTSTRAP_POOL_LOCK:
        if _BOOTSTRAP_POOL is not None and _BOOTSTRAP_POOL_META['workers']==int(workers):
            return _BOOTSTRAP_POOL,False,0.0
        if _BOOTSTRAP_POOL is not None:
            try: _BOOTSTRAP_POOL.shutdown(wait=False,cancel_futures=True)
            except Exception: pass
            _BOOTSTRAP_POOL=None
        import multiprocessing as mp
        t0=time.perf_counter()
        pool=ProcessPoolExecutor(max_workers=int(workers),mp_context=mp.get_context('spawn'),initializer=_bootstrap_worker_init)
        _BOOTSTRAP_POOL=pool
        _BOOTSTRAP_POOL_META.update({'workers':int(workers),'created_at':datetime.now().isoformat(),'create_ms':round((time.perf_counter()-t0)*1000.0,3),'method':'spawn','first_task_spawn_ms':None})
        return pool,True,_BOOTSTRAP_POOL_META['create_ms']

def _parallel_bootstrap_rows(samples,task_spec,a,c0,r0,common_plane,reference_plane,workers):
    """Map bootstrap replicates to the persistent process pool; fall back to serial on any
    pool failure (recorded, never silently dropped)."""
    meta={'method':'process_pool','workers':int(workers),'fallback_serial':False,'pool_error':None}
    payload_bytes=int(sum(s.nbytes for s in samples))
    try:
        t0=time.perf_counter()
        pool,created,create_ms=_get_bootstrap_pool(workers)
        meta['pool_created_this_request']=bool(created); meta['pool_create_ms']=create_ms
        payloads=[(sub,task_spec['radius'],task_spec['prior'],task_spec['mode'],a,c0,r0,common_plane,reference_plane) for sub in samples]
        t_submit=time.perf_counter()
        futures=[pool.submit(_bootstrap_task,payload) for payload in payloads]
        submit_ms=round((time.perf_counter()-t_submit)*1000.0,3)
        rows=[f.result() for f in futures]
        meta['submit_ms']=submit_ms; meta['collect_ms']=round((time.perf_counter()-t_submit)*1000.0,3)-submit_ms
        meta['payload_bytes']=payload_bytes
        comp=[r.pop('_worker_compute_ms') for r in rows]
        meta['worker_compute_sum_ms']=round(float(sum(comp)),3)
        meta['worker_compute_max_ms']=round(float(max(comp)),3)
        with _BOOTSTRAP_POOL_LOCK:
            m=dict(_BOOTSTRAP_POOL_META)
        meta['pool']=m
        return rows,meta
    except Exception as exc:
        meta['fallback_serial']=True; meta['pool_error']='{}: {}'.format(type(exc).__name__,exc)
        rows=[_bootstrap_row(sub,task_spec['radius'],task_spec['prior'],task_spec['mode'],a,c0,r0,common_plane,reference_plane) for sub in samples]
        return rows,meta

def bootstrap_line_stability(p,a,runner,points,prior,common_plane,reference_plane,boots=12,workers=1,task_spec=None):
    """Compare actual line/plane intersections; never compare arbitrary p0 values."""
    rng=np.random.default_rng(7); c0,_=line_plane_intersection(p,a,*common_plane)
    angle=[]; visible=[]; fixed=[]; visible_failures=[]; reference_failures=[]
    try: r0,_=line_plane_intersection(p,a,*reference_plane); reference_initial_error=None
    except Exception as exc: r0=None; reference_initial_error=str(exc)
    samples=[points[rng.choice(len(points),max(80,int(.75*len(points))),replace=True)] for _ in range(boots)]
    def one_bootstrap(sub):
        try:
            pp,aa,_,_=runner(sub); aa=canonical(aa,prior)
            ci,_=line_plane_intersection(pp,aa,*common_plane)
            row={'angle':float(np.degrees(np.arccos(np.clip(abs(np.dot(a,aa)),-1,1)))),'visible':float(np.linalg.norm(ci-c0)),'fixed':None,'visible_error':None,'reference_error':None}
            if r0 is not None:
                try:
                    ri,_=line_plane_intersection(pp,aa,*reference_plane); row['fixed']=float(np.linalg.norm(ri-r0))
                except Exception as exc: row['reference_error']=str(exc)
            return row
        except Exception as exc:return {'angle':None,'visible':None,'fixed':None,'visible_error':str(exc),'reference_error':None}
    bootstrap_started=time.perf_counter()
    parallel_meta=None
    if int(workers)>1 and task_spec is not None:
        rows,parallel_meta=_parallel_bootstrap_rows(samples,task_spec,a,c0,r0,common_plane,reference_plane,workers)
        execution='process_pool'
    elif int(workers)>1:
        with ThreadPoolExecutor(max_workers=int(workers),thread_name_prefix='cylinder-bootstrap') as pool: rows=list(pool.map(one_bootstrap,samples))
        execution='threads'
    else:
        rows=[one_bootstrap(sub) for sub in samples]; execution='serial'
    for row in rows:
        if row['visible_error'] is not None:visible_failures.append(row['visible_error']);continue
        angle.append(row['angle']);visible.append(row['visible'])
        if row['fixed'] is not None:fixed.append(row['fixed'])
        elif row['reference_error'] is not None:reference_failures.append(row['reference_error'])
    return {"requested_count":boots,"visible_success_count":len(angle),"visible_failure_count":len(visible_failures),"visible_failures":visible_failures[:3],"bootstrap_wall_ms":(time.perf_counter()-bootstrap_started)*1000.0,
            "reference_success_count":len(fixed),"reference_failure_count":len(reference_failures)+(boots if r0 is None else 0),"reference_failures":([reference_initial_error] if reference_initial_error else [])+reference_failures[:3],
            "axis_angle_deg":angle,"visible_common_section_center_mm":visible,"fixed_reference_plane_point_mm":fixed,
            "common_plane":"through final visible-axis segment centre; normal=final axis", "fixed_plane":"configured chassis Z plane transformed into camera","workers":int(workers),
            "execution":execution,"parallel":parallel_meta}
def robust_fit(points,radius,prior=None,mode="cylinder_3d",reference_plane=None,boots=None,bootstrap_workers=None):
    if boots is None: boots=_BOOTSTRAP_BOOTS_DEFAULT
    if len(points)<80: raise ValueError("too few body points (<80)")
    if bootstrap_workers is None: bootstrap_workers=_BOOTSTRAP_WORKERS_DEFAULT
    task_spec={'radius':radius,'prior':prior,'mode':mode} if int(bootstrap_workers)>1 else None
    if mode=="prior_2d":
        if prior is None: raise ValueError("prior_2d requires --axis-prior in camera frame")
        base_solver=lambda x:fit_prior(x,radius,prior)
    else:
        def base_solver(x): return _cylinder_base_solver(x,radius)
    def estimate_full(x): return _estimate_full(x,radius,prior,mode)
    fit_started=time.perf_counter(); p,a,res,extra=estimate_full(points); main_fit_wall_ms=(time.perf_counter()-fit_started)*1000.0
    # recompute residual after canonical direction (line unchanged)
    res=np.linalg.norm(np.cross(points-p,a),axis=1)-radius
    inlier=np.abs(res)<=5.0
    ts=(points[inlier]-p)@a; visible_mid=float(np.median(ts)); common_plane=(p+visible_mid*a,a)
    if reference_plane is None: reference_plane=common_plane
    stability=bootstrap_line_stability(p,a,estimate_full,points,prior,common_plane,reference_plane,boots,workers=bootstrap_workers,task_spec=task_spec)
    stability['bootstrap_wall_ms']=float(stability.get('bootstrap_wall_ms',0.0))
    extra['main_fit_wall_ms']=main_fit_wall_ms
    extra['least_squares_calls_total']=int(extra.get('least_squares_calls',0))+int(extra.get('initial_fit',{}).get('least_squares_calls',0))
    extra['least_squares_nfev_total']=int(extra.get('least_squares_nfev',0))+int(extra.get('initial_fit',{}).get('least_squares_nfev',0))
    extra['least_squares_wall_ms_total']=float(extra.get('least_squares_wall_ms',0.0))+float(extra.get('initial_fit',{}).get('least_squares_wall_ms',0.0))
    stability["axis_direction_is_hard_constrained"]=(mode=="prior_2d")
    if mode=="prior_2d": stability["axis_angle_interpretation"]="0-degree bootstrap changes are constraint results, not independent measured axis accuracy"
    return p,a,res,inlier,extra,stability

def project(points,K):
    p=np.asarray(points); return np.c_[p[:,0]*K['fx']/p[:,2]+K['cx'],p[:,1]*K['fy']/p[:,2]+K['cy']]
def matrix_from_json(path):
    d=json.loads(Path(path).read_text(encoding='utf-8'))
    for key in ('T_robot_camera','matrix','transform'):
        if key in d:
            m=np.asarray(d[key],float)
            if m.size==16:return m.reshape(4,4),d
    m=np.asarray(d,float)
    if m.size==16:return m.reshape(4,4),d
    raise ValueError('transform JSON must contain a 4x4 T_robot_camera, matrix, or transform')
def flatten_recorded_joints(state):
    d=json.loads(Path(state).read_text(encoding='utf-8'))['joints_deg']
    return np.asarray(d['trunk']+d['head'],float)
def load_sample_extrinsic(path,sample_id,expected_state):
    doc=json.loads(Path(path).read_text(encoding='utf-8')); rows=[r for r in doc.get('rows',[]) if str(r.get('sample'))==str(sample_id)]
    if len(rows)!=1: raise ValueError("sample {} must have exactly one transform row, found {}".format(sample_id,len(rows)))
    row=rows[0]
    if row.get('camera_frame')!='head_camera_color_optical_frame' or row.get('base_frame')!='chassis_link': raise ValueError('unexpected transform frames: {} <- {}'.format(row.get('base_frame'),row.get('camera_frame')))
    T=np.asarray(row.get('matrix_4x4'),float)
    if T.shape!=(4,4) or not np.allclose(T[3],[0,0,0,1],atol=1e-9): raise ValueError('invalid homogeneous matrix_4x4')
    R=T[:3,:3]
    if not np.allclose(R.T@R,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(R),1.0,atol=1e-6): raise ValueError('invalid rotation in matrix_4x4')
    saved=np.asarray(row.get('joint_values_deg'),float); recorded=flatten_recorded_joints(expected_state)
    if saved.shape!=recorded.shape or not np.allclose(saved,recorded,atol=1e-3): raise ValueError('transform row joint_values_deg do not match this sample robot_state.json')
    if not np.allclose(T[:3,3]*1000,np.asarray(row.get('translation_mm'),float),atol=1e-3): raise ValueError('matrix metre translation does not match translation_mm')
    return T,row
def validate_handeye(path):
    d=json.loads(Path(path).read_text(encoding='utf-8'))
    if d.get('base_frame')!='chassis_link' or d.get('camera_frame')!='head_camera_color_optical_frame':raise ValueError('handeye metadata does not identify chassis_link/head_camera_color_optical_frame')
    if d.get('calibration_type')!='eye_in_hand':raise ValueError('unexpected handeye calibration type')
    return {'path':str(Path(path).resolve()),'schema':d.get('schema'),'calibration_type':d.get('calibration_type'),'camera_frame':d.get('camera_frame'),'base_frame':d.get('base_frame'),'rms_px':d.get('intrinsics',{}).get('rms_px'),'distortion_coefficients_present':bool(d.get('intrinsics',{}).get('distortion_coefficients'))}
def chassis_plane_in_camera(T,z_ref_mm):
    """T is chassis<-camera, with metre translation; return plane in camera mm."""
    R=T[:3,:3]; t=T[:3,3]*1000; normal=R.T@np.array([0.,0.,1.]); point=R.T@(np.array([0.,0.,z_ref_mm])-t)
    return point,norm(normal)
def to_chassis(p_camera_mm,a_camera,T):
    R=T[:3,:3];t=T[:3,3]*1000;return R@p_camera_mm+t,R@a_camera
def to_camera(p_chassis_mm,T):
    R=T[:3,:3];t=T[:3,3]*1000;return R.T@(p_chassis_mm-t)
def fixed_height(p,a,args):
    """No visible-point statistic enters a physical fixed-height reference."""
    if args.height_mode=='none':return None,None,None
    if args.height_mode=='robot_z':
        if args.body_z_mm is None or args.t_robot_camera_path is None:raise ValueError('robot_z needs --body-z-mm and verified --t-robot-camera-path')
        T,_=matrix_from_json(args.t_robot_camera_path); pp=T[:3,:3]@p+T[:3,3]; aa=T[:3,:3]@a
        if abs(aa[2])<1e-8:raise ValueError('axis is parallel to configured robot z plane')
        q=pp+(args.body_z_mm-pp[2])/aa[2]*aa
        return q,canonical(aa,np.array([0.,0.,1.])),{'height_mode':'robot_z','height_value_mm':args.body_z_mm,'height_reference_source':str(args.t_robot_camera_path),'reference_frame':'robot_base'}
    if args.body_height_mm is None or args.support_plane_path is None:raise ValueError('support_relative needs --body-height-mm and --support-plane-path')
    d=json.loads(Path(args.support_plane_path).read_text(encoding='utf-8'))
    if not all(k in d for k in ('point_mm','normal','frame')):raise ValueError('support plane JSON needs point_mm, normal, frame')
    n=norm(d['normal'])
    if d['frame']=='camera':pp,aa=p,a
    elif d['frame']=='robot_base':
        if args.t_robot_camera_path is None:raise ValueError('robot-base support plane needs --t-robot-camera-path')
        T,_=matrix_from_json(args.t_robot_camera_path);pp=T[:3,:3]@p+T[:3,3];aa=T[:3,:3]@a
    else:raise ValueError('support plane frame must be camera or robot_base')
    den=np.dot(n,aa)
    if abs(den)<1e-8:raise ValueError('axis is parallel to configured support-relative plane')
    target=np.asarray(d['point_mm'],float)+args.body_height_mm*n;q=pp+np.dot(n,target-pp)/den*aa
    return q,canonical(aa,n),{'height_mode':'support_relative','height_value_mm':args.body_height_mm,'height_reference_source':str(args.support_plane_path),'reference_frame':d['frame']}
def write_ply(path,raw,fit,inliers):
    allp=np.vstack([raw,fit]); colors=np.vstack([np.tile([120,120,120],(len(raw),1)),np.where(inliers[:,None],[30,220,30],[30,30,230])])
    with Path(path).open('w',encoding='ascii') as f:
        f.write('ply\nformat ascii 1.0\nelement vertex %d\n'%len(allp)); f.write('property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
        for p,c in zip(allp,colors): f.write('%.5f %.5f %.5f %d %d %d\n'%(*p,*c))
def overlay(rgb,mask,fitmask,axispts,ref,K,status,metrics,path):
    out=rgb.copy(); out[mask]=(0.65*out[mask]+.35*np.array([255,160,0])).astype(np.uint8); out[fitmask]=(0.35*out[fitmask]+.65*np.array([0,210,0])).astype(np.uint8)
    contours,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out,contours,-1,(0,255,255),1); uv=project(axispts,K).astype(int); cv2.line(out,tuple(uv[0]),tuple(uv[1]),(0,0,255),3)
    if ref is not None:
        u=project(np.asarray(ref)[None],K)[0]
        if ref[2]>0 and 0<=u[0]<out.shape[1] and 0<=u[1]<out.shape[0]:cv2.drawMarker(out,tuple(u.astype(int)),(255,0,255),cv2.MARKER_CROSS,18,2)
    lines=[f"instance={metrics['instance_id']} index={metrics['array_index']} {status}",f"radial median/P90={metrics['radial_median_mm']:.2f}/{metrics['radial_p90_mm']:.2f} mm",'orange=mask green=fit-region; red=3D axis; magenta=fixed-height reference']
    for i,t in enumerate(lines):cv2.putText(out,t,(12,28+i*25),cv2.FONT_HERSHEY_SIMPLEX,.58,(0,0,0),3,cv2.LINE_AA);cv2.putText(out,t,(12,28+i*25),cv2.FONT_HERSHEY_SIMPLEX,.58,(255,255,255),1,cv2.LINE_AA)
    cv2.imwrite(str(path),out)
def _set_equal_3d_limits(ax, centre, half_extent):
    """Set genuine equal millimetre scale; avoids visually false tilt."""
    ax.set_xlim3d(centre[0]-half_extent,centre[0]+half_extent)
    ax.set_ylim3d(centre[1]-half_extent,centre[1]+half_extent)
    ax.set_zlim3d(centre[2]-half_extent,centre[2]+half_extent)
    ax.set_box_aspect((1,1,1))

def _draw_3d_panel(ax, raw, fit, result, plane, radius, title, centre, half_extent):
    """Detailed, review-oriented renderer for one fit mode in camera mm."""
    p,a,res,ins,extra,stab=result
    # Keep interaction responsive for large masks, while retaining every fit
    # point.  The raw mask cloud is only contextual and is uniformly sampled.
    raw_show=raw if len(raw)<=6000 else raw[np.linspace(0,len(raw)-1,6000,dtype=int)]
    ax.scatter(*raw_show.T,s=1,c='#909090',alpha=.16,depthshade=False,label='mask valid (context)')
    ax.scatter(*fit[~ins].T,s=3,c='#d62728',alpha=.55,depthshade=False,label='fit rejected')
    ax.scatter(*fit[ins].T,s=3,c='#2ca02c',alpha=.72,depthshade=False,label='fit inlier')
    ts=(fit[ins]-p)@a; rr=np.percentile(ts,[5,95]); line=p[None]+np.linspace(rr[0],rr[1],80)[:,None]*a
    ax.plot(*line.T,c='#0057b8',lw=2.8,label='fitted centre axis')
    u,v=basis(a); theta=np.linspace(0,2*np.pi,42); tt=np.linspace(rr[0],rr[1],2); th,tt=np.meshgrid(theta,tt)
    cyl=p[None,None,:]+tt[...,None]*a+radius*(np.cos(th)[...,None]*u+np.sin(th)[...,None]*v)
    ax.plot_wireframe(cyl[...,0],cyl[...,1],cyl[...,2],rstride=1,cstride=3,color='#00a6a6',alpha=.35,linewidth=.7)
    pc,pn=plane; pu,pv=basis(pn); grid=np.linspace(-100,100,2); xx,yy=np.meshgrid(grid,grid); surf=pc[None,None,:]+xx[...,None]*pu+yy[...,None]*pv
    ax.plot_surface(surf[...,0],surf[...,1],surf[...,2],alpha=.12,color='#b000b5',shade=False)
    try:
        ref,s=line_plane_intersection(p,a,*plane)
        ref_ok=stab['reference_success_count']==stab['requested_count'] and max(stab['fixed_reference_plane_point_mm'],default=float('inf'))<=MAX_BOOTSTRAP_REFERENCE_POINT_MM
        if ref_ok: ax.scatter(*ref,c='#b000b5',s=48,marker='P',depthshade=False,label='fixed-plane intersection')
        else: ax.scatter(*ref,c='#f08a00',s=48,marker='X',depthshade=False,label='diagnostic plane intersection')
    except Exception:
        ref=None; ref_ok=False
    ax.quiver(0,0,0,45,0,0,color='#d62728',arrow_length_ratio=.12)
    ax.quiver(0,0,0,0,45,0,color='#2ca02c',arrow_length_ratio=.12)
    ax.quiver(0,0,0,0,0,45,color='#0057b8',arrow_length_ratio=.12)
    max_angle=max(stab['axis_angle_deg'],default=float('nan')); max_visible=max(stab['visible_common_section_center_mm'],default=float('nan')); max_fixed=max(stab['fixed_reference_plane_point_mm'],default=float('nan'))
    ax.set_title(title+'\nP90 %.2f mm | span %.1f mm | boot angle %.2f deg\nvisible/fixed %.2f / %.2f mm%s'%(np.percentile(abs(res),90),rr[1]-rr[0],max_angle,max_visible,max_fixed,' | reference valid' if ref_ok else ' | diagnostic only'),fontsize=10)
    ax.set_xlabel('camera X (mm)');ax.set_ylabel('camera Y (mm)');ax.set_zlabel('camera Z (mm)')
    _set_equal_3d_limits(ax,centre,half_extent); ax.view_init(elev=21,azim=-68)
    ax.legend(loc='upper left',fontsize=7,framealpha=.82)
    return ref

def show3d_comparison(raw, fit, radius, prior, reference_plane, selected_mode):
    """Side-by-side prior/no-prior diagnostic with explicit wheel zoom.

    Matplotlib's normal left-drag rotation remains available.  Wheel events
    synchronously zoom both panels around the same scene centre, so a visual
    comparison cannot be distorted by unequal panel scales. Press R to reset.
    """
    import matplotlib.pyplot as plt
    results={}; failures={}
    for mode,mode_prior in (('cylinder_3d',None),('prior_2d',prior)):
        try: results[mode]=robust_fit(fit,radius,mode_prior,mode,reference_plane)
        except Exception as exc: failures[mode]=str(exc)
    all_points=[raw,fit]
    for value in results.values():
        p,a,_,ins,_,_=value; ts=(fit[ins]-p)@a; all_points.append(p[None]+np.percentile(ts,[5,95])[:,None]*a)
    cloud=np.vstack(all_points); lo=cloud.min(0); hi=cloud.max(0); centre=(lo+hi)/2; initial_half=max(float(np.max(hi-lo)*.58),80.0); state={'half':initial_half}
    fig=plt.figure(figsize=(16,8)); fig.suptitle('Bottle-axis diagnostic comparison — camera optical frame, millimetres\nMouse: left drag rotate; wheel: synchronized zoom; R: reset zoom. '+('Selected output: '+selected_mode),fontsize=12)
    axes=[fig.add_subplot(1,2,i+1,projection='3d') for i in range(2)]
    def redraw():
        for ax,mode in zip(axes,('cylinder_3d','prior_2d')):
            ax.cla()
            if mode in results:
                title=('NO PRIOR: cylinder_3d' if mode=='cylinder_3d' else 'CHASSIS +Z PRIOR: prior_2d')
                _draw_3d_panel(ax,raw,fit,results[mode],reference_plane,radius,title,centre,state['half'])
            else:
                ax.text2D(.08,.55,mode+' failed:\n'+failures[mode],transform=ax.transAxes,color='crimson',wrap=True)
                ax.set_title(mode+' unavailable');_set_equal_3d_limits(ax,centre,state['half'])
        fig.canvas.draw_idle()
    def on_scroll(event):
        if event.inaxes not in axes:return
        step=getattr(event,'step',0)
        factor=.82 if step>0 else 1.22
        state['half']=float(np.clip(state['half']*factor,25.0,5000.0));redraw()
    def on_key(event):
        if event.key and event.key.lower()=='r':state['half']=initial_half;redraw()
    fig.canvas.mpl_connect('scroll_event',on_scroll);fig.canvas.mpl_connect('key_press_event',on_key)
    redraw();fig.tight_layout(rect=(0,0,1,.92));plt.show()

def source_verify(result,vis,candidate_root):
    """Exact recreation of original all-instance visualization; returns unique RGB match."""
    target=cv2.imread(str(vis)); hits=[]
    for rgbpath in Path(candidate_root).rglob('head_rgb.jpg'):
        im=cv2.imread(str(rgbpath));
        if im is None or im.shape!=target.shape:continue
        colors=[(0,255,0),(0,128,255),(255,128,0),(255,0,255),(0,255,255),(255,0,0),(0,0,255)];ov=im.astype(np.float32)
        for i,d in enumerate(result['detections']):ov[rle(d)]=.55*ov[rle(d)]+.45*np.asarray(colors[i%7],np.float32)
        test=np.clip(ov,0,255).astype(np.uint8)
        for i,d in enumerate(result['detections']):
            x,y,w,h=map(lambda z:int(round(z)),d['bbox']);c=colors[i%7];cv2.rectangle(test,(x,y),(x+w,y+h),c,2);cv2.putText(test,f"#{i+1} {float(d['score']):.3f}",(x,max(0,y-6)),cv2.FONT_HERSHEY_SIMPLEX,.6,c,2,cv2.LINE_AA)
        info=f"Objects: {len(result['detections'])} | Model: {result.get('model','N/A')} | Threshold: {result.get('threshold','N/A')}";cv2.putText(test,info,(10,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2,cv2.LINE_AA);cv2.putText(test,info,(10,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,0),1,cv2.LINE_AA)
        diff=np.max(np.abs(test.astype(np.int16)-target.astype(np.int16)))
        if diff==0:hits.append(str(rgbpath.resolve()))
    return hits

def synthetic_test():
    # Analytic transform checks plus an executed, partial-arc cylinder fit.
    p=np.array([20.,-10.,350.]);a=norm([.18,.96,.22]); T1=np.eye(4);T1[:3,:3]=np.array([[0,-1,0],[1,0,0],[0,0,1]]);T1[:3,3]=[.12,-.07,1.4]
    T2=np.eye(4);T2[:3,:3]=np.array([[1,0,0],[0,0,-1],[0,1,0]]);T2[:3,3]=[-.2,.1,1.1]
    rows=[]
    for T in (T1,T2):
        plane=chassis_plane_in_camera(T,0.);q,s=line_plane_intersection(p,a,*plane);qc,ac=to_chassis(q,a,T);back=to_camera(qc,T)
        rows.append({'chassis_z_mm':float(qc[2]),'camera_roundtrip_mm':float(np.linalg.norm(back-q)),'translation_mm_check':float(np.linalg.norm((T[:3,3]*1000)-np.round(T[:3,3]*1000,12))),'tilted_axis_s':s})
    c0,_=line_plane_intersection(np.zeros(3),np.array([0,0,1.]),np.array([0,0,5.]),np.array([0,0,1.]));c1,_=line_plane_intersection(np.array([3.,0.,0.]),norm([1,0,1]),np.array([0,0,5.]),np.array([0,0,1.]))
    parallel=False
    try:line_plane_intersection(np.zeros(3),np.array([1.,0,0]),np.zeros(3),np.array([0,0,1.]))
    except ValueError:parallel=True
    sample_checks={}
    with tempfile.TemporaryDirectory() as td:
        td=Path(td);state=td/'robot_state.json';state.write_text(json.dumps({'joints_deg':{'trunk':[0,0,0,0],'head':[0,0]}}));row={'sample':'ok','camera_frame':'head_camera_color_optical_frame','base_frame':'chassis_link','matrix_4x4':np.eye(4).tolist(),'translation_mm':[0,0,0],'joint_values_deg':[0]*6}
        def rejects(doc,sid):
            f=td/'t.json';f.write_text(json.dumps(doc))
            try:load_sample_extrinsic(f,sid,state);return False
            except ValueError:return True
        sample_checks={'wrong_sample_rejected':rejects({'rows':[row]},'missing'),'duplicate_sample_rejected':rejects({'rows':[row,row]},'ok'),'invalid_rotation_rejected':rejects({'rows':[dict(row,matrix_4x4=np.diag([2,1,1,1]).tolist())]},'ok')}
    # Partial visible circular arc, axial span and outliers.  This executes the
    # same multistart/inlier/bootstrap route used by real prior_2d fitting.
    rng=np.random.default_rng(20260915); true_p=np.array([35.,-20.,500.]); true_a=norm([.12,.96,.25]); u,v=basis(true_a)
    th=np.linspace(-1.0,1.0,900); tt=rng.uniform(-70,70,len(th)); good=true_p+tt[:,None]*true_a+28.5*(np.cos(th)[:,None]*u+np.sin(th)[:,None]*v)+rng.normal(0,.35,(len(th),3))
    outliers=true_p+rng.uniform(-90,90,(120,3)); cloud=np.vstack([good,outliers])
    common=(true_p,true_a); ref_good=(true_p+30*true_a,true_a)
    fp,fa,fres,fins,fextra,fstab=robust_fit(cloud,28.5,true_a,'prior_2d',ref_good,boots=8)
    point_error=float(np.linalg.norm((fp-true_p)-np.dot(fp-true_p,true_a)*true_a)); angle_error=float(np.degrees(np.arccos(np.clip(abs(np.dot(fa,true_a)),-1,1))))
    # A perpendicular plane makes fixed-height intersection impossible, while
    # the visible-section bootstrap must remain available.
    _,_,_,_,_,parallel_stab=robust_fit(cloud,28.5,true_a,'prior_2d',(true_p,np.cross(true_a,u)),boots=6)
    # Direct line bootstrap with deliberately alternating directions verifies
    # that the angle statistic catches a >6 degree unstable solution.
    calls=[0]
    def unstable_runner(x):
        calls[0]+=1; aa=norm(true_a+(np.array([1.,0,0]) if calls[0]%2 else np.array([-1.,0,0]))*.22)
        return true_p,aa,np.zeros(len(x)),{}
    unstable=bootstrap_line_stability(true_p,true_a,unstable_runner,cloud,None,common,ref_good,boots=6)
    fit_checks={'partial_arc_point_error_mm':point_error,'partial_arc_axis_error_deg':angle_error,'multistart_candidate_count':fextra['candidate_count'],'partial_arc_bootstrap_visible_success':fstab['visible_success_count'],'parallel_reference_visible_success':parallel_stab['visible_success_count'],'parallel_reference_success':parallel_stab['reference_success_count'],'unstable_axis_max_angle_deg':max(unstable['axis_angle_deg'])}
    ok=all(abs(x['chassis_z_mm'])<1e-9 and x['camera_roundtrip_mm']<1e-9 for x in rows) and abs(c0[2]-5)<1e-9 and abs(c1[2]-5)<1e-9 and parallel and all(sample_checks.values()) and point_error<3 and angle_error<1e-6 and fextra['candidate_count']>=8 and fstab['visible_success_count']==8 and parallel_stab['visible_success_count']==6 and parallel_stab['reference_success_count']==0 and max(unstable['axis_angle_deg'])>MAX_BOOTSTRAP_AXIS_ANGLE_DEG
    print(jdump({'synthetic_test':'PASS' if ok else 'FAIL','cases':rows,'common_plane_intersections':[c0,c1],'near_parallel_rejected':parallel,'transform_validation':sample_checks,'cylinder_and_bootstrap':fit_checks,'note':'prior_2d direction is hard constrained; its zero bootstrap angle is not independent measured accuracy'}));return 0 if ok else 2

def make_all_instance_overview(rgb_path,result_path,camera_path,records,out_path,label,mode):
    """One auditable RGB overview per group/mode; axes are real 3-D projections."""
    rgb=cv2.imread(str(rgb_path)); result=json.loads(Path(result_path).read_text(encoding='utf-8')); K,_=camera_from_json(camera_path)
    out=rgb.copy()
    for rec in records:
        d=result['detections'][rec['instance_id']-1]; m=rle(d); colour=(0,200,0) if rec['axis_fit_valid'] else (0,150,255)
        out[m]=(0.72*out[m]+.28*np.asarray(colour)).astype(np.uint8)
        p=rec.get('axis_point_camera_mm'); a=rec.get('axis_direction_camera_up'); ts=rec.get('visible_axis_t_range_mm')
        if p is not None and a is not None and ts is not None:
            pts=np.asarray(p)[None]+np.asarray(ts)[:,None]*np.asarray(a); good=np.all(pts[:,2]>0)
            if good:
                uv=project(pts,K).astype(int);cv2.line(out,tuple(uv[0]),tuple(uv[1]),(0,0,255) if rec['axis_fit_valid'] else (0,128,255),2)
        x,y,w,h=map(int,d['bbox']); text='#{} A{} R{}'.format(rec['instance_id'],int(rec['axis_fit_valid']),int(rec['reference_point_valid']))
        cv2.putText(out,text,(x,max(14,y-5)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,0),3,cv2.LINE_AA);cv2.putText(out,text,(x,max(14,y-5)),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1,cv2.LINE_AA)
    title='{} {} | red=axis-valid  orange=axis-invalid  label A=axis R=reference'.format(label,mode)
    cv2.putText(out,title,(10,28),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),3,cv2.LINE_AA);cv2.putText(out,title,(10,28),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1,cv2.LINE_AA);cv2.imwrite(str(out_path),out)

def write_body_comparison_report(path,summary):
    lines=['# Bottle-body mask A/B offline axis-fit comparison','',
           'This is a diagnostic comparison only. No external axis ground truth is available; lower residual or more accepted instances does not establish millimetre accuracy.','',
           '## Provenance','']
    for g in summary['groups']:
        lines += ['- **{}**: source-verify `{}`; sample `{}`; RGB `{}`; depth `{}`; camera `{}` ({})'.format(g['label'],g['source_verify']['matches'],g['sample_id'],g['rgb_path'],g['depth_path'],g['camera_path'],g['camera_source'])]
    lines += ['','## Cross-group instance correspondence','','A and B were verified to originate from different RGB capture samples. Pixel IoU and #index correspondence are intentionally not used; no same-physical-bottle millimetre difference is claimed.','', '## Per-instance diagnostics','', '| Group | Mode | # | Axis | Ref | mask px | fit px | depth | inlier | median/P90 mm | span mm | boot angle deg | visible/fixed mm | extrapolation mm | rejection |','|---|---|---:|---|---|---:|---:|---:|---:|---|---:|---|---:|---:|---|']
    for r in summary['records']:
        s=r.get('bootstrap_stability',{}); ang=max(s.get('axis_angle_deg',[]) or [float('nan')]); vis=max(s.get('visible_common_section_center_mm',[]) or [float('nan')]); fix=max(s.get('fixed_reference_plane_point_mm',[]) or [float('nan')])
        lines.append('| {group} | {mode} | {instance_id} | {axis_fit_valid} | {reference_point_valid} | {mask_area_pixels} | {fit_mask_area_pixels} | {mask_valid_depth_ratio:.3f} | {inlier_ratio:.3f} | {radial_median_mm:.2f}/{radial_p90_mm:.2f} | {visible_axis_span_mm:.1f} | {ang:.2f} | {vis:.2f}/{fix:.2f} | {reference_extrapolation_from_visible_segment_mm} | {reason} |'.format(ang=ang,vis=vis,fix=fix,reason='; '.join(r.get('rejection_reasons',[])).replace('|','/'),**r))
    lines += ['','## Diagnostic conclusion','']
    for group in ('A','B'):
        prior=[r for r in summary['records'] if r['group']==group and r['mode']=='prior_2d']; cyl=[r for r in summary['records'] if r['group']==group and r['mode']=='cylinder_3d']
        lines.append('- **{}**: prior_2d accepted {}/{}, cylinder_3d accepted {}/{}; median P90 is {:.2f} mm vs {:.2f} mm.'.format(group,sum(r['axis_fit_valid'] for r in prior),len(prior),sum(r['axis_fit_valid'] for r in cyl),len(cyl),float(np.median([r['radial_p90_mm'] for r in prior])),float(np.median([r['radial_p90_mm'] for r in cyl]))))
    b7=[r for r in summary['records'] if r['group']=='B' and r['mode']=='cylinder_3d' and r['instance_id']==7]
    if b7:
        s=b7[0].get('bootstrap_stability',{}); lines.append('- **B #7 warning**: prior_2d passes only under the hard direction assumption, while cylinder_3d rejects its visible-section stability (max {:.2f} mm). Treat this narrow mask as unsuitable for a locating point until mask purity/depth support is independently confirmed.'.format(max(s.get('visible_common_section_center_mm',[]) or [float('nan')])))
    lines += ['','No external axis ground truth is present. These statements support better cylinder-model compatibility and internal geometric consistency only; they do not validate real-world millimetre accuracy.']
    lines += ['','## Interpretation limits','','- `bottle_body` uses a uniform edge erosion and depth-cluster filter, with no hidden 12/88 image-y trim. Inspect each `mask_fit_regions.png` before attributing a residual change to segmentation.','- `prior_2d` uses a hard chassis-up direction; zero bootstrap direction change is not measured visual accuracy.','- `Z_REF=1200 mm` is a shared diagnostic chassis plane, not a ground, box-bottom, or grasp-height claim.','- B uses explicitly recorded calibration K because its capture directory lacks `camera.json`; this is documented in `summary.json` and remains a provenance limitation.']
    Path(path).write_text('\n'.join(lines)+'\n',encoding='utf-8')

def run_body_mask_comparison(args):
    """Read-only orchestrator for the two supplied body-mask result sets."""
    timestamp=datetime.now().strftime('%Y%m%d_%H%M%S'); root=DEFAULT_BODY_MASK_COMPARISON_ROOT/timestamp;root.mkdir(parents=True,exist_ok=False)
    groups=[
        {'label':'A','result_path':DEFAULT_BODY_RESULT_A,'expected_count':6},
        {'label':'B','result_path':DEFAULT_BODY_RESULT_B,'expected_count':7},
    ]
    summary={'created_at':timestamp,'mask_kind':'bottle_body','mask_erosion_pixels':args.mask_erosion_pixels,'z_ref_mm':1200.0,'groups':[],'records':[],'cross_group_correspondence':{'status':'not_attempted','reason':'A and B source-verify to different RGB capture samples; pixel IoU/#index matching would not establish physical identity.'}}
    for group in groups:
        result=json.loads(group['result_path'].read_text(encoding='utf-8')); vis=group['result_path'].parent/'visualizations/head_rgb_vis.png'; hits=source_verify(result,vis,args.candidate_root)
        if len(hits)!=1: raise ValueError('{} source verification must yield exactly one RGB, got {}'.format(group['label'],hits))
        rgb_path=Path(hits[0]); sample=rgb_path.parent.name; data_dir=rgb_path.parent; depth_path=data_dir/'head_depth_aligned.npy'; local_camera=data_dir/'camera.json'
        if not depth_path.exists() or not (data_dir/'head_camera_metadata.json').exists() or not (data_dir/'robot_state.json').exists(): raise ValueError('{} source capture missing RGB-D metadata/state'.format(group['label']))
        if local_camera.exists(): camera_path=local_camera; camera_source='capture camera.json'
        else:
            camera_path=args.handeye_path; camera_source='calibration intrinsics fallback: capture camera.json missing; same serial/profile/frame documented'
            _,cdoc=camera_from_json(camera_path)
            if cdoc.get('camera',{}).get('serial_number')!='CP26363000CH' or cdoc.get('intrinsics',{}).get('image_size')!=[1280,720]: raise ValueError('B calibration fallback does not verify same camera serial/profile')
        depth=np.load(str(depth_path),mmap_mode='r')
        if tuple(depth.shape)!=(720,1280) or str(depth.dtype)!='float32': raise ValueError('{} depth shape/dtype is not confirmed aligned 720x1280 float32'.format(group['label']))
        if len(result['detections'])!=group['expected_count']: raise ValueError('{} expected {} detections, found {}'.format(group['label'],group['expected_count'],len(result['detections'])))
        group_info={'label':group['label'],'result_json':str(group['result_path']),'source_verify':{'matches':hits,'method':'pixel-exact reconstruction of saved visualisation'},'sample_id':sample,'rgb_path':str(rgb_path),'depth_path':str(depth_path),'camera_path':str(camera_path),'camera_source':camera_source,'depth_dtype':'float32','depth_unit':'millimetre','detection_count':len(result['detections'])}
        summary['groups'].append(group_info)
        for mode in ('prior_2d','cylinder_3d'):
            mode_root=root/group['label']/mode; mode_root.mkdir(parents=True,exist_ok=True); records=[]
            for iid in range(1,len(result['detections'])+1):
                before={p.resolve() for p in mode_root.glob('*_instance_*/axis_fit.json')}
                cmd=[sys.executable,str(Path(__file__).resolve()),'--result-json',str(group['result_path']),'--rgb-path',str(rgb_path),'--depth-path',str(depth_path),'--camera-path',str(camera_path),'--instance-id',str(iid),'--output-root',str(mode_root),'--sample-id',sample,'--fit-mode',mode,'--axis-prior-mode','chassis_up','--z-ref-mm','1200','--mask-kind','bottle_body','--mask-erosion-pixels',str(args.mask_erosion_pixels)]
                done=subprocess.run(cmd,capture_output=True,text=True)
                after=[p for p in mode_root.glob('*_instance_*/axis_fit.json') if p.resolve() not in before]
                if done.returncode!=0 or len(after)!=1: raise RuntimeError('{} {} #{} failed: {}'.format(group['label'],mode,iid,done.stderr[-1000:]))
                rec=json.loads(after[0].read_text(encoding='utf-8'))
                for key,value in {'mask_area_pixels':0,'fit_mask_area_pixels':0,'mask_valid_depth_ratio':float('nan'),'inlier_ratio':float('nan'),'radial_median_mm':float('nan'),'radial_p90_mm':float('nan'),'visible_axis_span_mm':float('nan'),'reference_extrapolation_from_visible_segment_mm':None}.items(): rec.setdefault(key,value)
                rec.update({'group':group['label'],'mode':mode,'output_dir':str(after[0].parent)});records.append(rec);summary['records'].append(rec)
            make_all_instance_overview(rgb_path,group['result_path'],camera_path,records,mode_root/'all_instances_axis_overview.png',group['label'],mode)
    (root/'summary.json').write_text(jdump(summary),encoding='utf-8');write_body_comparison_report(root/'comparison_report.md',summary);print(root);print(jdump({'root':str(root),'groups':summary['groups'],'record_count':len(summary['records'])}));return 0

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--result-json',type=Path,default=DEFAULT_RESULT_JSON);ap.add_argument('--rgb-path',type=Path,default=DEFAULT_RGB_PATH);ap.add_argument('--depth-path',type=Path,default=DEFAULT_DEPTH_PATH);ap.add_argument('--camera-path',type=Path,default=DEFAULT_CAMERA_PATH);ap.add_argument('--cad-path',type=Path,default=DEFAULT_CAD_PATH);ap.add_argument('--instance-id',type=int,default=DEFAULT_INSTANCE_ID);ap.add_argument('--output-root',type=Path,default=DEFAULT_OUTPUT_ROOT);ap.add_argument('--depth-scale',type=float,default=DEFAULT_DEPTH_SCALE);ap.add_argument('--body-radius-mm',type=float,default=DEFAULT_BODY_RADIUS_MM);ap.add_argument('--extrinsics-path',type=Path,default=DEFAULT_EXTRINSICS_PATH);ap.add_argument('--handeye-path',type=Path,default=DEFAULT_HANDEYE_PATH);ap.add_argument('--sample-id',default=DEFAULT_SAMPLE_ID);ap.add_argument('--z-ref-mm',type=float,default=DEFAULT_Z_REF_MM);ap.add_argument('--axis-prior-mode',choices=['chassis_up','manual','none'],default=DEFAULT_AXIS_PRIOR_MODE);ap.add_argument('--axis-prior',default=DEFAULT_AXIS_PRIOR);ap.add_argument('--fit-mode',choices=['prior_2d','cylinder_3d'],default=DEFAULT_FIT_MODE);ap.add_argument('--mask-kind',choices=['whole_bottle','bottle_body'],default=DEFAULT_MASK_KIND);ap.add_argument('--mask-erosion-pixels',type=int,default=DEFAULT_MASK_EROSION_PIXELS);ap.add_argument('--body-mask-comparison',action='store_true',help='run A/B bottle-body batches, both modes, source verification and report');ap.add_argument('--show-3d',action='store_true',default=DEFAULT_SHOW_3D);ap.add_argument('--synthetic-test',action='store_true');ap.add_argument('--verify-source',action='store_true');ap.add_argument('--candidate-root',type=Path,default=PROJECT/'data/20260915');args=ap.parse_args()
    if args.synthetic_test:return synthetic_test()
    if args.body_mask_comparison:return run_body_mask_comparison(args)
    result=json.loads(args.result_json.read_text(encoding='utf-8'))
    if args.verify_source:
        vis=args.result_json.parent/'visualizations/head_rgb_vis.png';hits=source_verify(result,vis,args.candidate_root);print(jdump({'verified_rgb_matches':hits,'count':len(hits),'method':'pixel-exact reconstruction of saved original SAM3 visualization'}));return 0 if len(hits)==1 else 3
    run=args.output_root/(datetime.now().strftime('%Y%m%d_%H%M%S')+f'_instance_{args.instance_id:03d}');run.mkdir(parents=True,exist_ok=False)
    report={'inputs':{k:str(getattr(args,k)) if getattr(args,k) is not None else None for k in ('result_json','rgb_path','depth_path','camera_path','cad_path','extrinsics_path','handeye_path')},'cad':read_obj_extent(args.cad_path),'instance_id':args.instance_id,'array_index':args.instance_id-1,'axis_fit_valid':False,'reference_point_valid':False,'rejection_reasons':[],'limitations':['raw RGB is projected/backprojected by pinhole K; calibrated distortion is currently not applied, so no repeated undistortion is performed']}
    try:
        if not all([args.rgb_path,args.depth_path,args.camera_path,args.body_radius_mm,args.extrinsics_path]):raise ValueError('missing verified RGB/depth/camera/extrinsics/body-radius input')
        if args.instance_id<1 or args.instance_id>len(result['detections']):raise ValueError('instance-id outside detections array')
        rgb=cv2.imread(str(args.rgb_path));depth=np.load(str(args.depth_path)).astype(float)*args.depth_scale;K,_=camera_from_json(args.camera_path);d=result['detections'][args.instance_id-1];mask=rle(d)
        if rgb is None or rgb.shape[:2]!=mask.shape or depth.shape!=mask.shape:raise ValueError(f'size mismatch RGB={None if rgb is None else rgb.shape[:2]}, depth={depth.shape}, mask={mask.shape}')
        sample=args.sample_id or args.rgb_path.parent.name; handeye=validate_handeye(args.handeye_path);T,row=load_sample_extrinsic(args.extrinsics_path,sample,args.rgb_path.parent/'robot_state.json'); R=T[:3,:3];up_cam=norm(R.T@np.array([0.,0.,1.]));ref_plane=chassis_plane_in_camera(T,args.z_ref_mm)
        raw,_,_=backproject(mask,depth,K);eroded_mask=eroded(mask,args.mask_erosion_pixels);fm=body_filter(mask,depth,args.mask_erosion_pixels,args.mask_kind);fit,_,_=backproject(fm,depth,K)
        if args.fit_mode=='prior_2d':
            prior=up_cam if args.axis_prior_mode=='chassis_up' else (parse_vec(args.axis_prior) if args.axis_prior_mode=='manual' else None)
            if prior is None:raise ValueError('prior_2d requires chassis_up or explicit manual camera prior')
        else: prior=None
        p,a,res,ins,extra,stab=robust_fit(fit,args.body_radius_mm,prior,args.fit_mode,ref_plane);a=canonical(a,up_cam)
        ts=(fit-p)@a;q=np.percentile(ts[ins],[5,95]); span=float(q[1]-q[0]); med=float(np.median(abs(res)));p90=float(np.percentile(abs(res),90))
        maxvis=max(stab['visible_common_section_center_mm']) if stab['visible_common_section_center_mm'] else float('inf')
        maxref=max(stab['fixed_reference_plane_point_mm']) if stab['fixed_reference_plane_point_mm'] else float('inf')
        maxangle=max(stab['axis_angle_deg']) if stab['axis_angle_deg'] else float('inf')
        visible_boot_ok=stab['visible_success_count']==stab['requested_count']
        angle_required=args.fit_mode=='cylinder_3d'
        angle_ok=(not angle_required) or maxangle<=MAX_BOOTSTRAP_AXIS_ANGLE_DEG
        ambiguity=bool(extra.get('candidate_solution_ambiguity',False))
        axis_ok=bool(len(fit)>=MIN_BODY_FIT_POINTS and span>=MIN_VISIBLE_AXIS_SPAN_MM and ins.mean()>=.50 and med<=3 and p90<=8 and visible_boot_ok and maxvis<=MAX_BOOTSTRAP_VISIBLE_CENTER_MM and angle_ok and not ambiguity)
        ref=None;refch=None;s=None;ref_intersection_error=None
        try:
            ref,s=line_plane_intersection(p,a,*ref_plane);pch,ach=to_chassis(p,a,T);refch,_=line_plane_intersection(pch,ach,np.array([0.,0.,args.z_ref_mm]),np.array([0.,0.,1.]))
        except Exception as exc: ref_intersection_error=str(exc)
        ref_boot_ok=stab['reference_success_count']==stab['requested_count']
        ref_ok=bool(axis_ok and ref is not None and ref_boot_ok and maxref<=MAX_BOOTSTRAP_REFERENCE_POINT_MM)
        projection={'in_front':None,'in_image':False}
        if ref is not None:
            projection['in_front']=bool(ref[2]>0)
            if projection['in_front']:
                uv=project(ref[None],K)[0];projection['pixel_uv']=uv;projection['in_image']=bool(0<=uv[0]<rgb.shape[1] and 0<=uv[1]<rgb.shape[0])
        reasons=[]
        if not axis_ok:reasons.append('axis gate: fit_points>=%d, axial_span>=%.1fmm, inlier>=0.50, median<=3mm, P90<=8mm, complete visible common-section bootstrap<=%.1fmm%s, no near-equivalent multistart centre ambiguity'%(MIN_BODY_FIT_POINTS,MIN_VISIBLE_AXIS_SPAN_MM,MAX_BOOTSTRAP_VISIBLE_CENTER_MM,(', cylinder_3d bootstrap angle<=%.1fdeg'%MAX_BOOTSTRAP_AXIS_ANGLE_DEG) if angle_required else ''))
        if ref_intersection_error:reasons.append('reference point unavailable: '+ref_intersection_error)
        elif not ref_boot_ok:reasons.append('reference point unavailable: fixed plane bootstrap intersections incomplete ({}/{})'.format(stab['reference_success_count'],stab['requested_count']))
        elif axis_ok and not ref_ok:reasons.append('reference gate: fixed chassis-Z-plane bootstrap<=%.1fmm'%MAX_BOOTSTRAP_REFERENCE_POINT_MM)
        report.update({'sample_id':sample,'capture_timestamp':json.loads((args.rgb_path.parent/'head_camera_metadata.json').read_text(encoding='utf-8')).get('camera_frame_captured_at'),'output_frame':'head_camera_color_optical_frame','output_unit':'mm','extrinsics_source':str(args.extrinsics_path),'handeye_validation':handeye,'extrinsics_row':{'sample':sample,'camera_frame':row['camera_frame'],'base_frame':row['base_frame'],'translation_input_unit':'m','translation_internal_unit':'mm','joint_match_verified':True},'mask_kind':args.mask_kind,'mask_erosion_pixels':args.mask_erosion_pixels,'eroded_mask_area_pixels':int(eroded_mask.sum()),'fit_mask_area_pixels':int(fm.sum()),'fit_mask_selection':'body mask: edge erosion plus depth-cluster filter; no image top/bottom trim' if args.mask_kind=='bottle_body' else 'whole bottle: edge erosion, 12/88 bbox-y trim plus depth-cluster filter','reference_frame':'chassis_link','reference_z_mm':args.z_ref_mm,'reference_z_configuration_note':'configured uniform diagnostic chassis Z={} mm; not measured ground, box bottom, or recommended grasp height'.format(args.z_ref_mm),'fit_mode':extra['method'],'fit_candidate_diagnostics':extra,'axis_prior_mode':args.axis_prior_mode,'axis_prior_camera':prior,'axis_prior_assumption':'bottle approximately upright and chassis approximately level' if prior is not None else None,'axis_direction_stability_required':angle_required,'axis_direction_stability_threshold_deg':MAX_BOOTSTRAP_AXIS_ANGLE_DEG if angle_required else None,'axis_direction_stability_note':'prior_2d bootstrap direction is hard constrained and is not independent measured accuracy' if not angle_required else 'cylinder_3d bootstrap direction change is gated','axis_point_camera_mm':p,'axis_direction_camera_up':canonical(a,up_cam),'reference_point_camera_mm':ref if ref_ok else None,'reference_point_chassis_mm':refch if ref_ok else None,'reference_intersection_diagnostic_camera_mm':ref,'reference_intersection_diagnostic_chassis_mm':refch,'reference_extrapolation_from_visible_segment_mm':float(min(abs(s-q[0]),abs(s-q[1])) if s is not None and (s<q[0] or s>q[1]) else 0.0) if s is not None else None,'visible_axis_t_range_mm':q,'visible_axis_span_mm':span,'radial_median_mm':med,'radial_p90_mm':p90,'inlier_ratio':float(ins.mean()),'fit_inlier_count':int(ins.sum()),'fit_region_point_count':len(fit),'raw_valid_point_count':len(raw),'mask_area_pixels':int(mask.sum()),'mask_valid_depth_ratio':float(np.count_nonzero(mask&np.isfinite(depth)&(depth>0))/mask.sum()),'bootstrap_stability':stab,'reference_projection':projection,'axis_fit_valid':axis_ok,'reference_point_valid':ref_ok,'rejection_reasons':reasons})
        status='AXIS+REFERENCE ACCEPTED' if ref_ok else ('AXIS ACCEPTED; REFERENCE INVALID' if axis_ok else 'DIAGNOSTIC / REJECTED')
        axispts=p[None]+q[:,None]*a;overlay(rgb,mask,fm,axispts,ref if ref_ok else None,K,status,{'instance_id':args.instance_id,'array_index':args.instance_id-1,'radial_median_mm':med,'radial_p90_mm':p90},run/'rgb_axis_overlay.png');save_mask_regions(rgb,mask,eroded_mask,fm,run/'mask_fit_regions.png',args.mask_kind);write_ply(run/'axis_points.ply',raw,fit,ins)
        if args.show_3d:
            visual_prior=up_cam if args.axis_prior_mode=='chassis_up' else (parse_vec(args.axis_prior) if args.axis_prior_mode=='manual' else None)
            show3d_comparison(raw,fit,args.body_radius_mm,visual_prior,ref_plane,args.fit_mode)
    except Exception as e: report['rejection_reasons']=[str(e)];report['limitations'].append('No geometric axis/reference is accepted when provenance, transform or quality gates are incomplete.')
    (run/'axis_fit.json').write_text(jdump(report),encoding='utf-8');(run/'README.md').write_text('# Offline bottle-axis diagnostic\n\nInputs are read-only. `axis_points.ply`: gray=mask valid, green=fit inlier, red=fit rejected.\n\nThe configured chassis `Z_REF` plane is used only to define an axis intersection. The saved reference point is null unless both the axis and the fixed-plane stability gate pass; it is never derived from mask or visible-point median. The default 1200 mm is a uniform diagnostic configuration, not measured ground/box height or a recommended grasp height.\n',encoding='utf-8');print(run);print(jdump(report));return 0
if __name__=='__main__':sys.exit(main())
