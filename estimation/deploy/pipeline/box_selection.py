"""Conservative two-stage bin selection; public geometry stays in full-image space.

This module owns the reusable 2-D left/right box policy and container-ROI
membership checks.  It deliberately does not perform 3-D localization.
"""
from itertools import combinations
import os
import cv2
import numpy as np


BOX_PROMPT = os.environ.get('DEFAULT_BOX_PROMPT', 'each individual open cardboard box')
BOX_THRESHOLD = 0.5

def rect(d):
    x, y, w, h = map(float, d['bbox'])
    return (x, y, x+w, y+h)

def _intersection_area(a, b):
    x1=max(a[0],b[0]); y1=max(a[1],b[1])
    x2=min(a[2],b[2]); y2=min(a[3],b[3])
    return max(0.0,x2-x1)*max(0.0,y2-y1)

def _coverage(outer, inner):
    """Fraction of ``inner`` bbox covered by ``outer`` bbox."""
    area=max(0.0,(inner[2]-inner[0])*(inner[3]-inner[1]))
    return _intersection_area(outer,inner)/area if area else 0.0

def choose_pair(detections, shape):
    h, w = shape[:2]
    candidates = []
    broad = []
    for index, d in enumerate(detections):
        x1,y1,x2,y2 = rect(d)
        bw,bh=x2-x1,y2-y1
        if not np.isfinite([x1,y1,x2,y2]).all() or bw<=0 or bh<=0:
            continue
        if bh >= .25*h and bw >= .18*w:
            if bw > .65*w:
                broad.append((x1,y1,x2,y2))
            elif bw*bh >= .06*w*h:
                candidates.append((index, (max(0,x1),max(0,y1),min(w,x2),min(h,y2))))
    if not candidates:
        raise ValueError('no_cardboard_box_detected')
    if len(candidates) == 1:
        c = candidates[0]
        return [c[0]], [list(c[1])]
    pairs=[]
    for a,b in combinations(candidates,2):
        left,right=sorted((a,b),key=lambda p:p[1][0])
        l,r=left[1],right[1]
        overlap=max(0,min(l[3],r[3])-max(l[1],r[1]))
        if overlap < .65*min(l[3]-l[1],r[3]-r[1]):
            continue
        if abs(r[0]-l[2]) > .1*w or min(l[2],r[2])-max(l[0],r[0]) > .1*min(l[2]-l[0],r[2]-r[0]):
            continue
        area=(l[2]-l[0])*(l[3]-l[1])+(r[2]-r[0])*(r[3]-r[1])
        pairs.append((area,left,right))
    if not pairs:
        raise ValueError('no_separable_left_right_box_pair')
    pairs.sort(key=lambda p:p[0],reverse=True)
    area,left,right=pairs[0]
    if len(pairs)>1 and pairs[1][0] >= .85*area:
        raise ValueError('ambiguous_box_pair_or_shelf')
    l,r=left[1],right[1]
    # Prefer two separable small boxes when a broad proposal contains both.
    # SAM3 can return the two physical boxes plus one proposal covering their
    # union.  A broad proposal is ignored only in that specific case; a broad
    # proposal elsewhere still keeps the conservative rejection behavior.
    broad_without_merged=[]
    for b in broad:
        left_coverage=_coverage(b,l)
        right_coverage=_coverage(b,r)
        if left_coverage > .70 and right_coverage > .70:
            continue
        broad_without_merged.append(b)
    if any((b[2]-b[0])*(b[3]-b[1]) >= .8*area for b in broad_without_merged):
        raise ValueError('merged_box_proposal')
    split=(l[2]+r[0])/2
    return [left[0],right[0]], [list((l[0],l[1],min(l[2],split),l[3])),
                                  list((max(r[0],split),r[1],r[2],r[3]))]

def filter_instances(detections, roi, shape, decode, min_inside=.9):
    kept, rejected, _ = filter_instances_detailed(
        detections, roi, shape, decode, min_inside=min_inside)
    # Preserve the original public helper's exact object values for legacy
    # callers; the detailed API carries the new traceability fields.
    original=[]
    for item in kept:
        upstream=int(item['upstream_instance_id'])
        original.append(detections[upstream-1])
    return original, rejected

def filter_instances_detailed(detections, roi, shape, decode, min_inside=.9,
                              reject_crop_boundary=True):
    """Keep complete masks assigned to ``roi`` and return per-instance audit.

    ``upstream_instance_id`` always refers to the 1-based order returned by the
    second SAM3 call.  Kept masks are not intersected with the ROI.
    """
    h,w=shape[:2]
    x1,y1,x2,y2=roi
    region=np.zeros((h,w),dtype=bool)
    region[int(np.ceil(y1)):int(np.floor(y2)),int(np.ceil(x1)):int(np.floor(x2))]=True
    kept,rejected=[],[]; audit=[]
    for i,d in enumerate(detections):
        upstream_id=int(d.get('upstream_instance_id',i+1)); row={
            'upstream_instance_id':upstream_id,
            'score':float(d.get('score',-1)),
        }
        try:
            mask=decode(d['segmentation'])
            if mask is None or mask.shape != (h,w) or not mask.any():
                raise ValueError('invalid_mask')
            yy,xx=np.nonzero(mask)
            ratio=float(np.count_nonzero(mask & region)/np.count_nonzero(mask))
            inside=x1<=float(xx.mean())<x2 and y1<=float(yy.mean())<y2
            boundary=bool(d.get('crop_boundary_touched',False))
            row.update(mask_centroid_xy=[float(xx.mean()),float(yy.mean())],
                       centroid_inside_roi=bool(inside),inside_ratio=ratio,
                       crop_boundary_touched=boundary)
            if inside and ratio >= min_inside and not (reject_crop_boundary and boundary):
                item=dict(d);item['upstream_instance_id']=upstream_id
                item['filtered_instance_id']=len(kept)+1;kept.append(item)
                row.update(kept=True,reason='kept')
            else:
                reasons=[]
                if not inside:reasons.append('centroid_outside_selected_box')
                if ratio < min_inside:reasons.append('inside_ratio_below_threshold')
                if reject_crop_boundary and boundary:reasons.append('touches_inference_crop_boundary')
                row.update(kept=False,reason=';'.join(reasons))
                rejected.append(dict(index=upstream_id,upstream_instance_id=upstream_id,
                                     reason=row['reason'],inside_ratio=ratio))
        except (ValueError,KeyError,TypeError,IndexError) as error:
            row.update(kept=False,reason='invalid_mask',detail=str(error))
            rejected.append(dict(index=upstream_id,upstream_instance_id=upstream_id,
                                 reason='invalid_mask',detail=str(error)))
        audit.append(row)
    return kept,rejected,audit



def run_pipeline(image_path, infer, decode, shape, target_box=1,
                 box_prompt=BOX_PROMPT, box_threshold=BOX_THRESHOLD,
                 min_inside=.9, **kwargs):
    if target_box not in (1,2):
        raise ValueError('target_box must be 1 (left) or 2 (right)')
    if not 0 < min_inside <= 1:
        raise ValueError('min_inside must be in (0, 1]')
    audit=dict(target_box=target_box,box_prompt=box_prompt,box_threshold=box_threshold,
               min_mask_inside_ratio=min_inside,coordinate_frame='original_image',
               selection_policy='largest_separable_horizontal_pair')
    def failed(reason):
        audit['reason']=reason
        return dict(ok=False,num_detections=0,detections=[],box_selection=audit)
    box_kwargs=dict(kwargs,prompt=box_prompt,threshold=box_threshold,return_vis=False)
    boxes=infer(image_path,**box_kwargs)
    if boxes is None or boxes.get('ok') is False:
        return failed('box_inference_failed')
    audit['box_candidates']=[dict(bbox=d['bbox'],score=d.get('score')) for d in boxes.get('detections',[])]
    try:
        indices,rois=choose_pair(boxes.get('detections',[]),shape)
    except ValueError as error:
        return failed(str(error))
    target_idx = 0 if len(rois) == 1 else target_box - 1
    audit.update(selected_box_detection_index=indices[target_idx]+1,selected_roi_xyxy=rois[target_idx])
    # Never request a server visualization that would still show rejected objects.
    objects=infer(image_path,**dict(kwargs,return_vis=False))
    if objects is None or objects.get('ok') is False:
        return failed('object_inference_failed')
    kept,rejected=filter_instances(objects.get('detections',[]),rois[target_idx],shape,decode,min_inside)
    audit.update(raw_object_count=len(objects.get('detections',[])),rejected=rejected,status='ok')
    for key in ('visualization_path','vis_base64','visualization_base64','output_json_path'):
        result.pop(key,None)
    return result

def select_front(detections, front):
    if not front: return [], 'front rule missing: provide front_axis_chassis, front_origin_chassis and front_band_mm'
    v = np.asarray(front['front_axis_chassis'], float); n = float(np.linalg.norm(v))
    axis = (v / n).tolist() if n > 1e-12 else [1.0, 0.0, 0.0]
    origin = np.asarray(front.get('front_origin_chassis', [0, 0, 0]), float)
    band = float(front['front_band_mm']); candidates = []; excluded = []
    for filtered_id, d in enumerate(detections, 1):
        i = int(d.get('upstream_instance_id', filtered_id))
        q = d.get('_median_chassis_point')
        if q is None: excluded.append((i, 'no valid depth centroid for front rule')); continue
        delta = float(np.dot(np.asarray(q) - origin, axis))
        if abs(delta) <= band: candidates.append((i, float(d.get('score', -1)), delta, filtered_id))
        else: excluded.append((i, 'outside configured front band'))
    candidates.sort(key=lambda x: (-x[1], x[0]))
    return candidates, {'axis_chassis': axis, 'origin_chassis': origin.tolist(), 'band_mm': band, 'excluded': excluded}

def apply_front_rule(items, shape, depth, K, T_m, front_rule, decoder, geometry_cache):
    """Optionally remove rear product masks using chassis-space depth band."""
    if not front_rule:
        return list(items), {'enabled': False, 'reason': 'front_rule_not_provided'}
    prepared = []
    for n, item in enumerate(items, 1):
        d = dict(item); d['upstream_instance_id'] = int(d.get('upstream_instance_id', n))
        if '_mask' not in d: d['_mask'] = decoder(d)
        raw, _, _, _ = geometry_cache.get_mask(d['_mask'], depth, K, T_m)
        d['_median_chassis_point'] = (np.median((T_m[:3, :3] @ raw.T).T + T_m[:3, 3] * 1000.0, axis=0).tolist() if len(raw) else None)
        prepared.append(d)
    candidates, info = select_front(prepared, front_rule)
    keep_ids = {x[0] for x in candidates}
    kept = [d for d in prepared if d['upstream_instance_id'] in keep_ids]
    audit = {'enabled': True, 'candidate_count': len(candidates), 'kept_upstream_instance_ids': [d['upstream_instance_id'] for d in kept], **info}
    return kept, audit

def parse_box_selection(req, class_cfg=None, default_box_prompt=BOX_PROMPT):
    class_cfg = class_cfg or {}
    raw = req.get('box_selection') or {}
    if not isinstance(raw, dict): raise ValueError('box_selection must be an object')
    allowed = {'enabled', 'target_box', 'box_prompt', 'box_threshold', 'target_threshold', 'min_inside_ratio'}
    unknown = sorted(set(raw) - allowed)
    if unknown: raise ValueError('unknown box_selection fields: ' + ','.join(unknown))
    if 'enabled' in raw and not isinstance(raw['enabled'], bool): raise ValueError('box_selection.enabled must be boolean')
    target_box = raw.get('target_box', 1)
    if isinstance(target_box, bool) or target_box not in (1, 2): raise ValueError('box_selection.target_box must be 1 or 2')
    box_prompt = raw.get('box_prompt', class_cfg.get('box_prompt', default_box_prompt))
    if not isinstance(box_prompt, str) or not box_prompt.strip(): raise ValueError('box_selection.box_prompt must be non-empty')

    def _val(k, d, low, high, open_low=False):
        v = float(raw.get(k, d))
        if not np.isfinite(v) or v > high or (v <= low if open_low else v < low): raise ValueError(f'box_selection.{k} out of range')
        return v

    box_threshold = _val('box_threshold', class_cfg.get('box_threshold', 0.5), 0.0, 1.0)
    target_threshold = _val('target_threshold', req.get('sam3_threshold', class_cfg.get('target_threshold', 0.5)), 0.0, 1.0)
    min_inside = _val('min_inside_ratio', 0.9, 0.0, 1.0, open_low=True)

    return {'enabled': True, 'target_box': int(target_box), 'box_prompt': box_prompt.strip(), 'box_threshold': box_threshold,
            'target_threshold': target_threshold, 'min_inside_ratio': min_inside,
            'stage_status': 'sku_box_selection_required'}

def box_overlay(rgb, detections, rois, indices, target_box, path, decoder=None):
    overlay = rgb.copy()
    for i, d in enumerate(detections, 1):
        try:
            m = decoder(d) if decoder else d.get('_mask')
            if m is not None:
                overlay[m] = (0.72 * overlay[m] + .28 * np.array([0, 190, 255])).astype(np.uint8)
        except Exception: pass
        x, y, w, h = map(int, d.get('bbox', [0, 0, 0, 0]))
        cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 170, 255), 2)
        cv2.putText(overlay, f'box#{i} {float(d.get("score", 0)):.3f}', (x, max(18, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3)
        cv2.putText(overlay, f'box#{i} {float(d.get("score", 0)):.3f}', (x, max(18, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    if rois:
        if len(rois) == 1:
            x1, y1, x2, y2 = map(int, rois[0])
            color = (0, 255, 0)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 3)
            cv2.putText(overlay, 'selected box', (x1, max(20, y1 + 20)), cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2)
        else:
            for side, roi in enumerate(rois, 1):
                x1, y1, x2, y2 = map(int, roi); color = (0, 255, 0) if side == target_box else (170, 170, 170)
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 3)
                cv2.putText(overlay, f'selected {"left" if side == 1 else "right"} box' if side == target_box else ('left' if side == 1 else 'right'), (x1, max(20, y1 + 20)), cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2)
    cv2.imwrite(str(path), overlay)
    return overlay


