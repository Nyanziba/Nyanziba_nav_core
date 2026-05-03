# Python API リファレンス（入口）

## 自動生成 API ドキュメント

Python API は Sphinx + autodoc で生成する。生成物は `docs/api/python/html/index.html`。

```bash
cd /path/to/texnitis_nav_core
pip install '.[docs]'
sphinx-build -b html sphinx docs/api/python/html
open docs/api/python/html/index.html
```

## モジュール構成（実装後）

```text
texnitis_nav_core/
  __init__.py        # _core を再エクスポート
  _core              # pybind11 ネイティブモジュール（M7 で追加）
  helpers            # numpy <-> GridMapView などの糖衣
  sim/               # 純 Python の 2D シミュレータ（M8 で追加）
    world
    kinematics
    disturbance
    runner
    metrics
    scenario
    visualize
```

## 最小サンプル（実装後）

実装後に `examples/python/astar_numpy_demo.py` の抜粋を貼る。
