"""Canonical public categories, with explicit legacy saved-file migration."""

SKU_TYPES = {'Avene': 'bottle', 'estee': 'box', 'origins': 'tube'}
LEGACY_SKUS = {value: key for key, value in SKU_TYPES.items()}


def sku_type(value):
    if isinstance(value, str) and value in LEGACY_SKUS:
        return value
    raise ValueError('商品类别无效：仅支持 bottle、box、tube；旧商品名称已停用')


def historical_type(value):
    """Only used when reading existing calibration or saved request files."""
    return sku_type(SKU_TYPES.get(value, value))


def local_target(value):
    return value if value == 'basket' else sku_type(value)


def shared_grasp_height(config):
    # Both arms use the same calibrated plane. Old config is read until migrated.
    if 'grasp_height_trunk_mm' in config:
        return config['grasp_height_trunk_mm']
    heights = config.get('grasp_height_trunk_mm_by_sku', {})
    return heights.get('bottle', heights.get('Avene'))


def validate_response_type(response, target, *, historical=False):
    """Fresh SKU responses must identify sku_typ; old files are read explicitly.

    Do not rewrite raw responses or silently infer a fresh response's category
    from its geometry. Historical mode is only for already saved local files.
    """
    if not isinstance(response, dict):
        raise ValueError('定位响应必须为 JSON 对象')
    if target == 'basket':
        if response.get('target_type') not in (None, 'basket'):
            raise ValueError('返回的目标类型与篮筐请求不一致')
        if response.get('sku_typ') is not None or response.get('sku_id') is not None:
            raise ValueError('篮筐响应不应包含商品类别')
        return
    expected = sku_type(target)
    if response.get('target_type') not in (None, 'sku'):
        raise ValueError('返回的目标类型与商品请求不一致')
    actual = response.get('sku_typ')
    if actual is None and historical:
        legacy = response.get('sku_id', response.get('class_name'))
        if legacy is not None and historical_type(legacy) != expected:
            raise ValueError('历史定位结果的商品类别不一致')
        return
    if actual != expected:
        raise ValueError(f'定位 sku_typ 缺失或不一致：期望 {expected}，收到 {actual!r}')
    if 'sku_id' in response:
        raise ValueError('新版定位响应不应包含旧 sku_id 字段')
    if response.get('class_name') not in (None, actual):
        raise ValueError('定位响应 class_name 与 sku_typ 不一致')
