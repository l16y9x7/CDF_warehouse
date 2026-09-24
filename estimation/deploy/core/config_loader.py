#!/usr/bin/env python3
"""JSON-backed configuration loader for SKU and Basket localization services.

Loads static defaults from deploy/config.json, applies environment variable
overrides, validates parameters, and exports canonical constants with 100%
backward compatibility.
"""
import json
import os
from pathlib import Path
from typing import Any, Dict, Optional
import numpy as np

DEPLOY_DIR: Path = Path(__file__).resolve().parent.parent
ROOT: Path = DEPLOY_DIR.parent
CONFIG_JSON_PATH: Path = DEPLOY_DIR / 'config.json'


def load_raw_json(path: Optional[Path] = None) -> Dict[str, Any]:
    """Loads and parses the static default JSON configuration file."""
    json_path = path or CONFIG_JSON_PATH
    if not json_path.is_file():
        raise FileNotFoundError(f"Configuration JSON not found at: {json_path}")
    with open(json_path, 'r', encoding='utf-8') as f:
        return json.load(f)


def build_config(
    raw_cfg: Optional[Dict[str, Any]] = None,
    env: Optional[Dict[str, str]] = None
) -> Dict[str, Any]:
    """Builds a configuration mapping from static defaults and environment overrides.

    Args:
        raw_cfg: Optional pre-loaded static config dict. If None, reads from config.json.
        env: Optional environment mapping for testing overrides. If None, uses os.environ.

    Returns:
        Dict containing all resolved configurations, paths, prompts, and class settings.
    """
    if raw_cfg is None:
        raw_cfg = load_raw_json()
    e = os.environ if env is None else env

    # Service & Network
    default_host = e.get('HOST', raw_cfg.get('service', {}).get('default_host', '0.0.0.0'))
    default_port = int(e.get('PORT', raw_cfg.get('service', {}).get('default_port', 25540)))
    max_body_bytes = int(e.get('MAX_BODY_BYTES', raw_cfg.get('service', {}).get('max_body_bytes', 32 * 1024 * 1024)))
    artifact_subdir = raw_cfg.get('service', {}).get('artifact_subdir', 'requests')
    artifact_root = Path(e.get('AXIS_SERVICE_OUTPUT', str(ROOT / artifact_subdir)))
    save_request_inputs = e.get(
        'SAVE_REQUEST_INPUTS',
        '1' if raw_cfg.get('service', {}).get('save_request_inputs', True) else '0'
    ).strip().lower() in ('1', 'true', 'yes', 'on')

    enable_3d_render = e.get(
        'ENABLE_3D_RENDER',
        '1' if raw_cfg.get('service', {}).get('enable_3d_render', True) else '0'
    ).strip().lower() in ('1', 'true', 'yes', 'on')
    async_3d_render = e.get(
        'ASYNC_3D_RENDER',
        '1' if raw_cfg.get('service', {}).get('async_3d_render', True) else '0'
    ).strip().lower() in ('1', 'true', 'yes', 'on')

    # SAM3 Upstream
    sam3_backend = e.get('SAM3_BACKEND', raw_cfg.get('sam3', {}).get('backend', 'legacy_18003'))
    if sam3_backend not in ('legacy_18003', 'multipart_segment'):
        raise ValueError('SAM3_BACKEND must be legacy_18003 or multipart_segment')

    sam3_urls = raw_cfg.get('sam3', {}).get('urls', {})
    if sam3_backend == 'multipart_segment':
        default_sam3_url = sam3_urls.get('multipart_segment', 'http://127.0.0.1:25541/api/v1/segment')
    else:
        default_sam3_url = sam3_urls.get('legacy_18003', 'http://127.0.0.1:25551/infer')

    sam3_url = e.get('SAM3_URL', default_sam3_url)
    sam3_mask_threshold = float(e.get('SAM3_MASK_THRESHOLD', raw_cfg.get('sam3', {}).get('mask_threshold', 0.5)))
    sam3_timeout_s = float(e.get('SAM3_TIMEOUT_S', raw_cfg.get('sam3', {}).get('timeout_s', 180.0)))

    if not np.isfinite(sam3_mask_threshold) or not 0 <= sam3_mask_threshold <= 1:
        raise ValueError('SAM3_MASK_THRESHOLD must be in [0,1]')

    # Prompts
    prompts = raw_cfg.get('prompts', {})
    container_box_prompt = e.get('DEFAULT_BOX_PROMPT', prompts.get('container_box', 'each individual open cardboard box'))
    bottle_default_prompt = prompts.get('bottle', 'The main cylindrical body of each white bottle')
    box_default_prompt = prompts.get('box', 'All visible top surfaces of the blue green boxes')
    tube_default_prompt = prompts.get('tube', 'All individual gray green package ends')

    # Basket / FoundationPose
    basket_cfg = raw_cfg.get('basket', {})
    basket_default_prompt = basket_cfg.get('default_prompt', 'the central white plastic basket')
    basket_default_threshold = float(basket_cfg.get('default_threshold', 0.7))
    mesh_rel = basket_cfg.get('mesh_relative_path', 'cad/Basket/Basket.obj')
    basket_mesh_path = Path(e.get('BASKET_MESH_PATH', str(DEPLOY_DIR / mesh_rel)))
    basket_mesh_scale = float(e.get('BASKET_MESH_SCALE', basket_cfg.get('mesh_scale', 0.001)))
    basket_fp_url = e.get('BASKET_FP_URL', basket_cfg.get('foundationpose_url', 'http://127.0.0.1:25550/infer'))
    basket_fp_registered_cad = e.get(
        'BASKET_FP_REGISTERED_CAD',
        '1' if basket_cfg.get('foundationpose_registered_cad', False) else '0'
    ).strip().lower() in ('1', 'true', 'yes', 'on')

    # Front Rule Default
    front_rule_default = raw_cfg.get('front_rule_default', {
        'front_axis_chassis': [1.0, 0.0, 0.0],
        'front_origin_chassis': [0.0, 0.0, 0.0],
        'front_band_mm': 100000.0,
    })

    # Class configs
    classes = raw_cfg.get('classes', {})
    class_config = {
        'bottle': {
            'sam3_prompt': bottle_default_prompt,
            'box_prompt': container_box_prompt,
            'box_threshold': classes.get('bottle', {}).get('box_threshold', 0.5),
            'target_threshold': classes.get('bottle', {}).get('target_threshold', 0.5),
        },
        'box': {
            'sam3_prompt': box_default_prompt,
            'box_prompt': container_box_prompt,
            'box_threshold': classes.get('box', {}).get('box_threshold', 0.5),
            'target_threshold': classes.get('box', {}).get('target_threshold', 0.5),
        },
        'tube': {
            'sam3_prompt': tube_default_prompt,
            'box_prompt': container_box_prompt,
            'box_threshold': classes.get('tube', {}).get('box_threshold', 0.5),
            'target_threshold': classes.get('tube', {}).get('target_threshold', 0.2),
        },
    }

    # Algorithm script paths for reflection
    bottle_fit_path = DEPLOY_DIR / 'fit_bottle_axis.py'
    estee_fit_path = DEPLOY_DIR / 'fit_estee_box_top_surface.py'
    box_selection_path = DEPLOY_DIR / 'box_selection.py'
    tube_fit_path = DEPLOY_DIR / 'fit_tube_top_edge.py'
    front_panel_fit_path = DEPLOY_DIR / 'fit_front_panel_plane.py'
    box_geometric_path = DEPLOY_DIR / 'box_geometric_center.py'
    box_pipeline_path = DEPLOY_DIR / 'box_pipeline.py'

    return {
        'ROOT': ROOT,
        'DEPLOY_DIR': DEPLOY_DIR,
        'DEFAULT_HOST': default_host,
        'DEFAULT_PORT': default_port,
        'MAX_BODY_BYTES': max_body_bytes,
        'ARTIFACT_ROOT': artifact_root,
        'SAVE_REQUEST_INPUTS': save_request_inputs,
        'ENABLE_3D_RENDER': enable_3d_render,
        'ASYNC_3D_RENDER': async_3d_render,
        'SAM3_BACKEND': sam3_backend,
        'SAM3_URL': sam3_url,
        'SAM3_MASK_THRESHOLD': sam3_mask_threshold,
        'SAM3_TIMEOUT_S': sam3_timeout_s,
        'CONTAINER_BOX_PROMPT': container_box_prompt,
        'DEFAULT_BOX_PROMPT': container_box_prompt,
        'BOTTLE_DEFAULT_PROMPT': bottle_default_prompt,
        'BOX_DEFAULT_PROMPT': box_default_prompt,
        'TUBE_DEFAULT_PROMPT': tube_default_prompt,
        'BASKET_DEFAULT_PROMPT': basket_default_prompt,
        'BASKET_DEFAULT_THRESHOLD': basket_default_threshold,
        'BASKET_MESH_PATH': basket_mesh_path,
        'BASKET_MESH_SCALE': basket_mesh_scale,
        'BASKET_FP_URL': basket_fp_url,
        'BASKET_FP_REGISTERED_CAD': basket_fp_registered_cad,
        'FRONT_RULE_DEFAULT': front_rule_default,
        'CLASS_CONFIG': class_config,
        'BOTTLE_FIT_PATH': bottle_fit_path,
        'ESTEE_FIT_PATH': estee_fit_path,
        'BOX_SELECTION_PATH': box_selection_path,
        'TUBE_FIT_PATH': tube_fit_path,
        'FRONT_PANEL_FIT_PATH': front_panel_fit_path,
        'BOX_GEOMETRIC_PATH': box_geometric_path,
        'BOX_PIPELINE_PATH': box_pipeline_path,
    }


# Initialize canonical module-level exports
_CONFIG_MAP = build_config()

ROOT: Path = _CONFIG_MAP['ROOT']
DEPLOY_DIR: Path = _CONFIG_MAP['DEPLOY_DIR']
DEFAULT_HOST: str = _CONFIG_MAP['DEFAULT_HOST']
DEFAULT_PORT: int = _CONFIG_MAP['DEFAULT_PORT']
MAX_BODY_BYTES: int = _CONFIG_MAP['MAX_BODY_BYTES']
ARTIFACT_ROOT: Path = _CONFIG_MAP['ARTIFACT_ROOT']
SAVE_REQUEST_INPUTS: bool = _CONFIG_MAP['SAVE_REQUEST_INPUTS']
ENABLE_3D_RENDER: bool = _CONFIG_MAP['ENABLE_3D_RENDER']
ASYNC_3D_RENDER: bool = _CONFIG_MAP['ASYNC_3D_RENDER']
SAM3_BACKEND: str = _CONFIG_MAP['SAM3_BACKEND']
SAM3_URL: str = _CONFIG_MAP['SAM3_URL']
SAM3_MASK_THRESHOLD: float = _CONFIG_MAP['SAM3_MASK_THRESHOLD']
SAM3_TIMEOUT_S: float = _CONFIG_MAP['SAM3_TIMEOUT_S']
CONTAINER_BOX_PROMPT: str = _CONFIG_MAP['CONTAINER_BOX_PROMPT']
DEFAULT_BOX_PROMPT: str = _CONFIG_MAP['DEFAULT_BOX_PROMPT']
BOTTLE_DEFAULT_PROMPT: str = _CONFIG_MAP['BOTTLE_DEFAULT_PROMPT']
BOX_DEFAULT_PROMPT: str = _CONFIG_MAP['BOX_DEFAULT_PROMPT']
TUBE_DEFAULT_PROMPT: str = _CONFIG_MAP['TUBE_DEFAULT_PROMPT']
BASKET_DEFAULT_PROMPT: str = _CONFIG_MAP['BASKET_DEFAULT_PROMPT']
BASKET_DEFAULT_THRESHOLD: float = _CONFIG_MAP['BASKET_DEFAULT_THRESHOLD']
BASKET_MESH_PATH: Path = _CONFIG_MAP['BASKET_MESH_PATH']
BASKET_MESH_SCALE: float = _CONFIG_MAP['BASKET_MESH_SCALE']
BASKET_FP_URL: str = _CONFIG_MAP['BASKET_FP_URL']
BASKET_FP_REGISTERED_CAD: bool = _CONFIG_MAP['BASKET_FP_REGISTERED_CAD']
FRONT_RULE_DEFAULT: Dict[str, Any] = _CONFIG_MAP['FRONT_RULE_DEFAULT']
CLASS_CONFIG: Dict[str, Dict[str, Any]] = _CONFIG_MAP['CLASS_CONFIG']

BOTTLE_FIT_PATH: Path = _CONFIG_MAP['BOTTLE_FIT_PATH']
ESTEE_FIT_PATH: Path = _CONFIG_MAP['ESTEE_FIT_PATH']
BOX_SELECTION_PATH: Path = _CONFIG_MAP['BOX_SELECTION_PATH']
TUBE_FIT_PATH: Path = _CONFIG_MAP['TUBE_FIT_PATH']
FRONT_PANEL_FIT_PATH: Path = _CONFIG_MAP['FRONT_PANEL_FIT_PATH']
BOX_GEOMETRIC_PATH: Path = _CONFIG_MAP['BOX_GEOMETRIC_PATH']
BOX_PIPELINE_PATH: Path = _CONFIG_MAP['BOX_PIPELINE_PATH']
