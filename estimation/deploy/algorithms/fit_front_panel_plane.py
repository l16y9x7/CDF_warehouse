#!/usr/bin/env python3
"""Offline RGB-D front-panel plane probe; no SAM3 call and no robot action."""
import argparse, json, math, os, random
from pathlib import Path
import cv2, numpy as np
from pycocotools import mask as mask_utils

ROOT=Path(__file__).resolve().parents[2]
DEFAULT_RESULT_JSON=ROOT/'sam3/test/results/prompt_threshold_sweep/20260916_174724_929092/p00/125449063.json'
DEFAULT_RGB_PATH=ROOT/'data/20260915/125449063/head_rgb.jpg'
DEFAULT_DEPTH_PATH=ROOT/'data/20260915/125449063/head_depth_aligned.npy'
DEFAULT_HANDEYE_PATH=ROOT/'sam3/test/axis_fit_support/head_camera_transform_20260915/calibration/head_camera_handeye_20260914.json'
DEFAULT_TRANSFORMS_PATH=ROOT/'sam3/test/axis_fit_support/head_camera_transform_20260915/transforms_20260915.json'
DEFAULT_SAMPLE_ID='125449063'; DEFAULT_INSTANCE_ID=3
DEFAULT_OUTPUT_ROOT=ROOT/'sam3/test/front_panel_plane_results'

# Candidate knobs (defaults preserve the original behaviour exactly):
# - FRONT_PANEL_RANSAC_MAX_ITER: iteration budget for the front-panel RANSAC (was fixed 800).
#   With the same seed the first N samples are identical; only the best-consensus search depth changes.
# - FRONT_PANEL_RANSAC_SKIP_MEDIAN: skip the median tie-break for iterations that cannot win
#   (count strictly below the current best). Exact: the selected best plane is unchanged.
_FRONT_PANEL_RANSAC_MAX_ITER_DEFAULT=max(1,int(os.environ.get('FRONT_PANEL_RANSAC_MAX_ITER','800') or 800))
_FRONT_PANEL_RANSAC_SKIP_MEDIAN=os.environ.get('FRONT_PANEL_RANSAC_SKIP_MEDIAN','0').strip().lower() in ('1','true','yes','on')

def unit(v):
    v=np.asarray(v,float); n=float(np.linalg.norm(v))
    if n<1e-12: raise ValueError('zero vector')
    return v/n
def decode_rle(det):
    seg=det['segmentation']; rle={'size':seg['size'],'counts':seg['counts']}
    if isinstance(rle['counts'],str): rle['counts']=rle['counts'].encode('ascii')
    return mask_utils.decode(rle).astype(bool)
def load_K(path):
    doc=json.loads(Path(path).read_text(encoding='utf-8'))
    K=np.asarray(doc['intrinsics']['camera_matrix'],float)
    return K
def load_transform(path,sample):
    rows=json.loads(Path(path).read_text(encoding='utf-8')).get('rows',[])
    rows=[r for r in rows if str(r.get('sample'))==str(sample)]
    if len(rows)!=1: raise ValueError('transform sample must match exactly one row')
    T=np.asarray(rows[0]['matrix_4x4'],float)
    if T.shape!=(4,4) or not np.allclose(T[3],[0,0,0,1],atol=1e-8): raise ValueError('invalid transform')
    R=T[:3,:3]
    if not np.allclose(R.T@R,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(R),1,atol=1e-5): raise ValueError('invalid rotation')
    T=T.copy(); T[:3,3]*=1000.0
    return T,rows[0]
def ransac_plane(points, threshold=4.0, iterations=800, seed=7, skip_median=False):
    points=np.asarray(points,float)
    if len(points)<3: raise ValueError('insufficient points')
    rng=random.Random(seed); best=None
    for _ in range(iterations):
        i,j,k=rng.sample(range(len(points)),3)
        n=np.cross(points[j]-points[i],points[k]-points[i]); norm=np.linalg.norm(n)
        if norm<1e-7: continue
        n=n/norm; d=-float(n@points[i]); residual=np.abs(points@n+d); keep=residual<=threshold
        count=int(keep.sum())
        if skip_median and best is not None and count<best[0][0]: continue
        med=float(np.median(residual[keep])) if count else float('inf')
        key=(count,-med)
        if best is None or key>best[0]: best=(key,keep,n,d)
    if best is None: raise ValueError('RANSAC found no plane')
    keep=best[1]; q=points[keep]; center=q.mean(axis=0); _,_,vh=np.linalg.svd(q-center,full_matrices=False); n=unit(vh[-1]); d=-float(n@center)
    residual=np.abs(points@n+d); keep=residual<=threshold
    return {'normal':n,'d':d,'residual':residual,'keep':keep,'point':center}

def fit_front_panel(points_chassis, pixels, depth_axis_chassis, threshold_mm=4.0,
                    depth_window_mm=45.0, max_points=30000, seed=7):
    """Fit the near vertical panel from an existing box mask in chassis_link.

    depth_axis_chassis follows the convention robot/work side -> shelf interior;
    therefore the front panel is the robust minimum projection along this axis.
    """
    p=np.asarray(points_chassis,float); px=np.asarray(pixels,int); axis=unit(depth_axis_chassis)
    if len(p)<300: return {'valid':False,'reasons':['insufficient valid box-mask depth points']}
    projection=p@axis; near=float(np.percentile(projection,2.0)); slab=projection<=near+float(depth_window_mm)
    q=p[slab]; qpx=px[slab]
    if len(q)<300: return {'valid':False,'reasons':['insufficient near-depth points for front panel'],'near_projection_mm':near}
    if len(q)>max_points:
        take=np.linspace(0,len(q)-1,max_points).astype(int);q=q[take];qpx=qpx[take]
    fit=ransac_plane(q,threshold_mm,_FRONT_PANEL_RANSAC_MAX_ITER_DEFAULT,seed,skip_median=_FRONT_PANEL_RANSAC_SKIP_MEDIAN); n=unit(fit['normal'])
    if np.dot(n,-axis)<0:n=-n
    keep=fit['keep']; inliers=q[keep]; inlier_px=qpx[keep]
    up=np.array([0.,0.,1.]); verticality=float(np.degrees(np.arcsin(min(1.,abs(n@up)))))
    prior_angle=float(np.degrees(np.arccos(np.clip(abs(n@axis),-1.,1.))))
    residual=fit['residual'][keep]; horizontal=unit(np.cross(up,n))
    h=inliers@up; x=inliers@horizontal
    h_lo,h_hi=np.percentile(h,[2,98]);x_lo,x_hi=np.percentile(x,[2,98])
    top_band=h>=np.percentile(h,95);top_points=inliers[top_band]
    top_mid=(np.median(top_points,axis=0) if len(top_points) else None)
    reasons=[]
    if len(inliers)<300:reasons.append('insufficient front-plane inliers')
    if float(keep.mean())<0.15:reasons.append('front-plane inlier ratio below threshold')
    if np.median(residual)>2.0 or np.percentile(residual,90)>4.0:reasons.append('front-plane residual gate failed')
    if verticality>10.0:reasons.append('front plane is not sufficiently vertical to chassis up')
    if prior_angle>20.0:reasons.append('front-plane normal disagrees with configured shelf depth axis')
    if x_hi-x_lo<100.0 or h_hi-h_lo<30.0:reasons.append('front-plane support is too small')
    return {'valid':not reasons,'reasons':reasons,'point_chassis_mm':np.median(inliers,axis=0),
            'normal_chassis':n,'top_edge_midpoint_chassis_mm':top_mid,'inlier_points_chassis':inliers,
            'inlier_pixels_xy':inlier_px,'candidate_point_count':int(len(q)),'inlier_count':int(len(inliers)),
            'inlier_ratio':float(keep.mean()),'residual_median_mm':float(np.median(residual)),
            'residual_p90_mm':float(np.percentile(residual,90)),'verticality_error_deg':verticality,
            'normal_prior_angle_deg':prior_angle,'horizontal_span_mm':float(x_hi-x_lo),
            'vertical_span_mm':float(h_hi-h_lo),'near_projection_mm':near,
            'depth_window_mm':float(depth_window_mm),'ransac_iterations':int(_FRONT_PANEL_RANSAC_MAX_ITER_DEFAULT),
            'ransac_median_skip':bool(_FRONT_PANEL_RANSAC_SKIP_MEDIAN)}
def backproject(depth,mask,K,stride=2):
    ys,xs=np.where(mask & np.isfinite(depth) & (depth>0))
    if stride>1: ys=ys[::stride]; xs=xs[::stride]
    z=depth[ys,xs].astype(float); x=(xs-K[0,2])*z/K[0,0]; y=(ys-K[1,2])*z/K[1,1]
    return np.column_stack([x,y,z]),xs,ys
def write_ply(path,points,keep):
    points=np.asarray(points); keep=np.asarray(keep,bool); colors=np.where(keep[:,None],np.array([40,210,80]),np.array([210,70,50]))
    with Path(path).open('w',encoding='ascii') as f:
        f.write('ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n'.format(len(points)))
        for p,c in zip(points,colors): f.write('{:.5f} {:.5f} {:.5f} {} {} {}\n'.format(*p,*c))
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--result-json',type=Path,default=DEFAULT_RESULT_JSON); ap.add_argument('--rgb-path',type=Path,default=DEFAULT_RGB_PATH); ap.add_argument('--depth-path',type=Path,default=DEFAULT_DEPTH_PATH); ap.add_argument('--handeye-path',type=Path,default=DEFAULT_HANDEYE_PATH); ap.add_argument('--transforms-path',type=Path,default=DEFAULT_TRANSFORMS_PATH); ap.add_argument('--sample-id',default=DEFAULT_SAMPLE_ID); ap.add_argument('--instance-id',type=int,default=DEFAULT_INSTANCE_ID); ap.add_argument('--output-root',type=Path,default=DEFAULT_OUTPUT_ROOT); ap.add_argument('--mask-erosion-pixels',type=int,default=2); ap.add_argument('--ransac-threshold-mm',type=float,default=4.0); ap.add_argument('--max-points',type=int,default=30000); args=ap.parse_args()
    rgb=cv2.imread(str(args.rgb_path),cv2.IMREAD_COLOR); depth=np.load(args.depth_path,allow_pickle=False).astype(float); doc=json.loads(args.result_json.read_text(encoding='utf-8')); body=doc.get('body',doc); detections=body['detections']; iid=args.instance_id
    if iid<1 or iid>len(detections): raise ValueError('instance id out of range')
    mask=decode_rle(detections[iid-1]);
    if mask.shape!=depth.shape or rgb.shape[:2]!=depth.shape: raise ValueError('RGB/depth/mask dimensions differ')
    K=load_K(args.handeye_path); T,row=load_transform(args.transforms_path,args.sample_id); up_cam=unit(T[:3,:3].T@np.array([0.,0.,1.])); eroded=cv2.erode(mask.astype(np.uint8),np.ones((2*args.mask_erosion_pixels+1,)*2,np.uint8),iterations=1).astype(bool)
    points,xs,ys=backproject(depth,eroded,K,2)
    if len(points)>args.max_points:
        idx=np.linspace(0,len(points)-1,args.max_points).astype(int); points=points[idx]; xs=xs[idx]; ys=ys[idx]
    fit=ransac_plane(points,args.ransac_threshold_mm)
    n=fit['normal']; d=float(fit['d']);
    if np.dot(n,-fit['point'])<0: n=-n; d=-d
    verticality=float(np.degrees(np.arcsin(min(1.,abs(np.dot(n,up_cam)))))); q=points[fit['keep']]; heights=q@up_cam; horiz=unit(np.cross(up_cam,n)); hc=q@horiz; high=heights>=np.percentile(heights,95); edge=q[high]; top=np.mean(edge,axis=0) if len(edge) else None
    output=args.output_root/(str(args.sample_id)+'_instance_{:03d}'.format(iid)); output.mkdir(parents=True,exist_ok=True); cv2.imwrite(str(output/'mask_original.png'),mask.astype(np.uint8)*255); cv2.imwrite(str(output/'mask_eroded.png'),eroded.astype(np.uint8)*255); write_ply(output/'front_panel_points.ply',points,fit['keep'])
    overlay=rgb.copy()
    overlay[mask & ~eroded]=(60,60,60); overlay[eroded]=(0,180,0)
    # Show actual sampled plane points: green=inliers, red=RANSAC rejects.
    for px,py,ok in zip(xs.astype(int),ys.astype(int),fit['keep']):
        overlay[py,px]=(40,220,40) if ok else (0,0,220)
    if top is not None:
        u=int(np.median(xs[fit['keep']][high])); v=int(np.median(ys[fit['keep']][high])); cv2.circle(overlay,(u,v),10,(0,0,255),-1); cv2.putText(overlay,'top-edge candidate',(u+10,max(20,v-10)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
    cv2.imwrite(str(output/'front_panel_overlay.jpg'),overlay)
    report={'sample_id':args.sample_id,'instance_id':iid,'array_index':iid-1,'inputs':{k:str(Path(v).resolve()) for k,v in {'result_json':args.result_json,'rgb':args.rgb_path,'depth':args.depth_path,'handeye':args.handeye_path,'transforms':args.transforms_path}.items()},'output_frame':'head_camera_color_optical_frame','output_unit':'mm','mask_area_pixels':int(mask.sum()),'eroded_mask_area_pixels':int(eroded.sum()),'valid_depth_points':int(len(points)),'plane_valid':bool(len(q)>=3),'plane_point_camera_mm':fit['point'].tolist(),'plane_normal_camera':n.tolist(),'plane_equation_d_mm':d,'up_camera':up_cam.tolist(),'verticality_error_deg':verticality,'plane_residual_median_mm':float(np.median(fit['residual'][fit['keep']])), 'plane_residual_p90_mm':float(np.percentile(fit['residual'][fit['keep']],90)),'plane_inlier_count':int(fit['keep'].sum()),'plane_inlier_ratio':float(fit['keep'].mean()),'top_edge_candidate_camera_mm':top.tolist() if top is not None else None,'top_edge_candidate_count':int(len(edge)),'assumptions':['front-panel candidate reused existing box mask; no extra SAM3 call','verticality uses chassis +Z transformed into camera frame','top edge is a visible high quantile candidate, not a reconstructed hidden full-box edge'],'transform_row':row}
    (output/'front_panel_plane.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8'); print(json.dumps({'output_dir':str(output.resolve()),'plane_valid':report['plane_valid'],'valid_depth_points':report['valid_depth_points'],'inlier_count':report['plane_inlier_count'],'inlier_ratio':report['plane_inlier_ratio'],'median_residual_mm':report['plane_residual_median_mm'],'p90_residual_mm':report['plane_residual_p90_mm'],'verticality_error_deg':verticality,'top_edge_candidate_camera_mm':report['top_edge_candidate_camera_mm']},ensure_ascii=False,indent=2))

def fit_selected_box_front_panel(selected_box, rgb, depth, K, T_m, req, geometry_cache, path, estee_module=None, decoder=None):
    rule=req.get('front_rule')
    if not rule:return {'enabled':False,'valid':False,'reasons':['front_rule_not_provided']},None
    if decoder is not None:
        mask = decoder(selected_box)
    else:
        mask = selected_box.get('_mask')
        if mask is None:
            mask = decode_rle(selected_box)
    erosion=int(req.get('front_panel_erosion_pixels',2))
    inner=cv2.erode(mask.astype(np.uint8),np.ones((2*erosion+1,2*erosion+1),np.uint8)).astype(bool)
    pc,pixels,_,ch=geometry_cache.get_mask(inner,depth,K,T_m)
    axis=unit(rule['front_axis_chassis'])
    try:
        fit=fit_front_panel(ch,pixels,axis,
            threshold_mm=float(req.get('front_panel_ransac_threshold_mm',4.0)),
            depth_window_mm=float(req.get('front_panel_depth_window_mm',45.0)))
    except ValueError as exc:
        fit={'valid':False,'reasons':['front-panel fit failed: '+str(exc)]}
    audit={'enabled':True,'valid':bool(fit.get('valid')),'reasons':fit.get('reasons',[]),
           'mask_area_pixels':int(mask.sum()),'eroded_mask_area_pixels':int(inner.sum()),
           'valid_depth_points':int(len(pc))}
    for key in ('candidate_point_count','inlier_count','inlier_ratio','residual_median_mm','residual_p90_mm','verticality_error_deg','normal_prior_angle_deg','horizontal_span_mm','vertical_span_mm','near_projection_mm','depth_window_mm'):
        if key in fit:audit[key]=fit[key]
    overlay=rgb.copy(); overlay[mask]=(0.7*overlay[mask]+0.3*np.array([0,180,255])).astype(np.uint8)
    if fit.get('inlier_pixels_xy') is not None:
        pp=np.asarray(fit['inlier_pixels_xy'],int);overlay[pp[:,1],pp[:,0]]=(40,220,40)
    if fit.get('point_chassis_mm') is not None:
        pcam=estee_module.chassis_to_camera(np.asarray(fit['point_chassis_mm'])[None],T_m)[0] if estee_module else (T_m[:3,:3].T@(np.asarray(fit['point_chassis_mm'])-T_m[:3,3]*1000.0)).tolist()
        ncam=T_m[:3,:3].T@np.asarray(fit['normal_chassis'])
        audit.update({'plane_point_chassis_mm':fit['point_chassis_mm'],'plane_point_camera_mm':pcam,
                      'plane_normal_chassis':fit['normal_chassis'],'plane_normal_camera':ncam,
                      'plane_equation_chassis':'dot(normal_chassis, p - plane_point_chassis_mm) = 0'})
    top=fit.get('top_edge_midpoint_chassis_mm')
    if top is not None:
        topcam=estee_module.chassis_to_camera(np.asarray(top)[None],T_m)[0] if estee_module else (T_m[:3,:3].T@(np.asarray(top)-T_m[:3,3]*1000.0)).tolist()
        audit.update({'top_edge_midpoint_chassis_mm':top,'top_edge_midpoint_camera_mm':topcam})
        if topcam[2]>0:
            uv=estee_module.project(topcam[None],K)[0] if estee_module else [(topcam[0]*K['fx']/topcam[2]+K['cx']),(topcam[1]*K['fy']/topcam[2]+K['cy'])]
            if 0<=uv[0]<rgb.shape[1] and 0<=uv[1]<rgb.shape[0]:cv2.drawMarker(overlay,tuple(np.rint(uv).astype(int)),(0,0,255),cv2.MARKER_CROSS,20,2)
    cv2.imwrite(str(path),overlay)
    return audit,overlay

if __name__=='__main__': main()

