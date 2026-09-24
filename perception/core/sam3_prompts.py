"""SAM3 prompt configuration for 3.3 vision understanding APIs."""

from __future__ import annotations

import os
from typing import Final

LOCATE_BLUE_LIGHT: Final = "locate_blue_light"
LOCATE_BASKET: Final = "locate_basket"
LOCATE_CARTON_QR_CODE: Final = "locate_carton_qr_code"
LOCATE_SKU_QR_CODE: Final = "locate_sku_qr_code"
LOCATE_BASKET_BUTTON: Final = "locate_basket_button"
LOCATE_BASKET_ITEM: Final = "basket_locate_item"
RECOGNIZE_SKU_BARCODE: Final = "recognize_sku_barcode"

_DEFAULT_PROMPTS: dict[str, str] = {
    LOCATE_BLUE_LIGHT: "blue indicator light",
    LOCATE_BASKET: "white plastic basket with slotted sides",
    LOCATE_CARTON_QR_CODE: "white label with barcode",
    LOCATE_SKU_QR_CODE: "product barcode",
    LOCATE_BASKET_BUTTON: "colored round indicator button",
    LOCATE_BASKET_ITEM: "product in basket",
    RECOGNIZE_SKU_BARCODE: "product barcode",
}

_ENV_KEYS: dict[str, str] = {
    LOCATE_BLUE_LIGHT: "SUPERVISION_LOCATE_BLUE_LIGHT_PROMPT",
    LOCATE_BASKET: "SUPERVISION_LOCATE_BASKET_PROMPT",
    LOCATE_CARTON_QR_CODE: "SUPERVISION_LOCATE_CARTON_QR_CODE_PROMPT",
    LOCATE_SKU_QR_CODE: "SUPERVISION_RECOGNIZE_SKU_BARCODE_PROMPT",
    LOCATE_BASKET_BUTTON: "SUPERVISION_LOCATE_BASKET_BUTTON_PROMPT",
    LOCATE_BASKET_ITEM: "SUPERVISION_LOCATE_BASKET_ITEM_PROMPT",
    RECOGNIZE_SKU_BARCODE: "SUPERVISION_RECOGNIZE_SKU_BARCODE_PROMPT",
}


def default_sam3_prompt(api_name: str) -> str:
    if api_name not in _DEFAULT_PROMPTS:
        known = ", ".join(sorted(_DEFAULT_PROMPTS))
        raise KeyError(f"unknown API prompt: {api_name}; known: {known}")
    env_key = _ENV_KEYS[api_name]
    return os.getenv(env_key, _DEFAULT_PROMPTS[api_name])


def resolve_sam3_prompt(api_name: str, override: str | None = None) -> str:
    if override is not None and override.strip():
        return override.strip()
    return default_sam3_prompt(api_name)
