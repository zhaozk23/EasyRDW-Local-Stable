简单的 Python 版本：visibility polygon + VisPoly RDW

包含文件：
- visibility_polygon.py: 切片（slice）与可见性多边形计算
- vis_poly_rdw.py: RDW 逻辑的 Python 移植（set_gains / set_steer_target 等）
- vec2.py, geometry.py: 几何与向量工具

说明：这是一个“可运行/可读”的翻译，保留了原始 C++ 逻辑结构但省略或简化了某些细节（例如精细的 loss 计算、CGAL 布尔操作等）。

快速使用示例：
```python
from pasumi_py.vec2 import Vec2
from pasumi_py.vis_poly_rdw import VisPolyRdw

# 构造一个简单的矩形物理环境示例
env = { 'vertices': [Vec2(0,0), Vec2(10,0), Vec2(10,10), Vec2(0,10)], 'obstacles': [] }

# mock user/state 对象需要实现 state.get_phys_pos(), state.get_virt_pos(),
# get_phys_heading(), get_virt_heading(), physical_env(), virtual_env()

```
