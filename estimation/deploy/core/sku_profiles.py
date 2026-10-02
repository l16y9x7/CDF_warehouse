"""Resolve optional business SKU IDs to server-owned vision configuration."""
import json
import math
from pathlib import Path

from config_loader import SKU_PROFILES_PATH


PROFILE_FIELDS = {
    'name', 'notes', 'sku_typ', 'sam3_prompt', 'target_threshold',
    'max_mask_area_ratio', 'body_radius_mm',
}


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate SKU library key: {key}')
        result[key] = value
    return result


def load_sku_profiles(path=None):
    """Read a fresh, validated snapshot. Atomic file replacement enables hot updates."""
    path = Path(path) if path is not None else SKU_PROFILES_PATH
    with path.open(encoding='utf-8-sig') as stream:
        data = json.load(stream, object_pairs_hook=_unique_object)
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('SKU library schema_version must be 1')
    if not isinstance(data.get('revision'), str) or not data['revision'].strip():
        raise ValueError('SKU library revision must be a non-empty string')
    profiles = data.get('profiles')
    if not isinstance(profiles, dict):
        raise ValueError('SKU library profiles must be an object')
    for sku_id, profile in profiles.items():
        if not sku_id.strip() or sku_id != sku_id.strip() or not isinstance(profile, dict):
            raise ValueError(f'invalid SKU library entry: {sku_id!r}')
        unknown = set(profile) - PROFILE_FIELDS
        if unknown:
            raise ValueError(f'SKU {sku_id}: unknown fields: {sorted(unknown)}')
        if profile.get('sku_typ') not in ('bottle', 'box', 'tube'):
            raise ValueError(f'SKU {sku_id}: sku_typ must be bottle, box or tube')
        for key in ('name', 'sam3_prompt'):
            if not isinstance(profile.get(key), str) or not profile[key].strip():
                raise ValueError(f'SKU {sku_id}: {key} must be a non-empty string')
        for key in ('target_threshold', 'max_mask_area_ratio', 'body_radius_mm'):
            if key not in profile:
                continue
            value = profile[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'SKU {sku_id}: invalid {key}')
            if key == 'target_threshold':
                valid = 0 <= value <= 1
            elif key == 'max_mask_area_ratio':
                valid = 0 < value <= 1
            else:
                valid = value > 0 and profile['sku_typ'] == 'bottle'
            if not valid:
                raise ValueError(f'SKU {sku_id}: {key} out of range or incompatible with sku_typ')
    return data


def apply_sku_profile(req, path=None):
    """Apply profile fields before target normalization; unrelated request fields remain caller-owned.

    No ID (or null) preserves the legacy sku_typ contract. Explicit unknown IDs and
    type conflicts fail instead of silently applying a different product's settings.
    """
    sku_id = req.get('sku_id')
    if sku_id is None:
        req['_sku_profile'] = {'source': 'category_default', 'sku_id': None}
        return
    if not isinstance(sku_id, str) or not sku_id.strip():
        raise ValueError('sku_id must be a non-empty string when provided')
    sku_id = sku_id.strip()
    library = load_sku_profiles(path)
    profile = library['profiles'].get(sku_id)
    if profile is None:
        raise ValueError(f'unknown sku_id {sku_id!r}; add it to the server SKU library')
    supplied_type = req.get('sku_typ')
    if supplied_type is not None and supplied_type != profile['sku_typ']:
        raise ValueError(f'sku_typ {supplied_type!r} conflicts with sku_id {sku_id!r} ({profile["sku_typ"]})')
    raw_box = req.get('box_selection')
    if raw_box is not None and not isinstance(raw_box, dict):
        raise ValueError('box_selection must be an object')
    box = dict(raw_box or {})
    for key in ('target_threshold', 'max_mask_area_ratio'):
        if key in profile:
            box[key] = profile[key]
    req.update(sku_id=sku_id, sku_typ=profile['sku_typ'], sam3_prompt=profile['sam3_prompt'].strip())
    req['box_selection'] = box
    if 'body_radius_mm' in profile:
        req['body_radius_mm'] = profile['body_radius_mm']
    req['_sku_profile'] = {
        'source': 'sku_library', 'sku_id': sku_id, 'name': profile['name'],
        'library_revision': library['revision'],
        'applied_parameters': {key: profile[key] for key in profile if key not in ('name', 'notes')},
    }
