# SKU 配置重构说明

## 问题描述
原先 SKU 配置存在重复和不一致的问题：
- `configs/products.yaml` 包含商品信息（列表格式）
- `configs/workflows.yaml` 的 `skus` 字段为空，但代码从这里读取
- 导致运行时报错：`sku_id 3282779003131 未配置`

## 解决方案
实施方案 2：让 SKU 配置统一从 `products.yaml` 读取，避免重复配置。

### 修改内容

#### 1. `src/agent/config.py`
- 添加 `DEFAULT_PRODUCTS_PATH = Path("configs/products.yaml")`
- 重写 `load_sku_catalog()` 函数：
  - 默认从 `products.yaml` 读取
  - 将列表格式 `products` 转换为字典格式
  - 调用 `parse_sku_catalog()` 解析为 `dict[str, SkuSpec]`

#### 2. `src/agent/application.py`
- 修改 `build_application()` 函数第 119 行
- 从 `load_sku_catalog(workflows_path)` 改为 `load_sku_catalog()`
- 使用默认的 products.yaml 路径

### 配置文件格式

**products.yaml** (商品信息的唯一来源):
```yaml
products:
  - sku_id: "3282779003131"
    name: 雅漾舒护活泉水
    sku_typ: bottle
    hand: RIGHT
    # ... 其他展示字段
```

**workflows.yaml** (保持 skus 为空):
```yaml
skus: {}  # SKU 配置已迁移到 products.yaml
```

### 验证

运行测试：
```bash
python test_sku_loading.py
```

预期输出：
```
✓ 所有 SKU 配置加载成功
✓ SKU 查询成功: 雅漾舒护活泉水
✓ 正确处理缺失的 SKU
所有测试通过！
```

### 向后兼容性

- `load_sku_catalog()` 仍然接受可选的 `path` 参数
- 如有特殊需求，可以传入自定义配置文件路径
- `debug/catalog.py` 中的 `_SKU_CATALOG = load_sku_catalog()` 无需修改

### 添加新商品

只需在 `configs/products.yaml` 的 `products` 列表中添加：
```yaml
products:
  - sku_id: "新商品ID"
    name: 商品名称
    sku_typ: bottle|box|tube  # 必填
    hand: LEFT|RIGHT          # 必填
    description: 描述
    category: 分类
    image_url: 图片路径
```

重启服务后自动生效。
