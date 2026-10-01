#!/usr/bin/env python
"""验证 SKU 配置从 products.yaml 正确加载"""
import sys
from pathlib import Path

# 添加 src 到 path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from agent.config import load_sku_catalog
from agent.skus import sku_spec

def test_load_sku_catalog():
    """测试从 products.yaml 加载 SKU 配置"""
    catalog = load_sku_catalog()

    assert len(catalog) == 3, f"Expected 3 SKUs, got {len(catalog)}"

    # 验证所有预期的 SKU 都存在
    expected_skus = ["3282779003131", "887167608641", "7173342765403"]
    for sku_id in expected_skus:
        assert sku_id in catalog, f"SKU {sku_id} not found in catalog"

    print("✓ 所有 SKU 配置加载成功")

def test_sku_spec_lookup():
    """测试 SKU 查询功能"""
    catalog = load_sku_catalog()

    # 测试成功查询
    spec = sku_spec(catalog, "3282779003131")
    assert spec.sku_typ == "bottle"
    assert spec.hand.name == "RIGHT"
    assert spec.name == "雅漾舒护活泉水"

    print(f"✓ SKU 查询成功: {spec.name}")

def test_missing_sku():
    """测试查询不存在的 SKU"""
    catalog = load_sku_catalog()

    try:
        sku_spec(catalog, "nonexistent")
        assert False, "Should raise ValueError for missing SKU"
    except ValueError as e:
        assert "未配置" in str(e)
        print(f"✓ 正确处理缺失的 SKU: {e}")

if __name__ == "__main__":
    test_load_sku_catalog()
    test_sku_spec_lookup()
    test_missing_sku()
    print("\n所有测试通过！")
