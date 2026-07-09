# 手眼标定工具包

本仓库是一套用于机械臂手眼标定的实验代码，主要包含两条路线：

1. **官方棋盘格方法**：采集棋盘格图像和机械臂末端位姿，通过 OpenCV `calibrateHandEye` 求解手眼矩阵。
2. **Jing 小方块方法**：使用相机检测到的小方块中心 3D 坐标、机械臂末端位姿，以及小方块中心在机械臂基坐标系下的真实坐标，通过最小二乘拟合相机到机械臂末端的变换。

其中：

- `compute_in_hand.py` 是官方眼在手上棋盘格标定流程。
- `jing_hand_eye_compute.py` 是自定义的小方块最小二乘标定流程。
- `detect_red_block.py` 保留旧文件名，但现在已经改成“任意颜色小方块检测”，不再只识别红色。
- `compute_in_hand_12data.py` 是用 Jing 方法整理出的 12 组样例数据验证脚本。

## 目录结构

```text
hand_eye_calibration/
  collect_data.py              # 官方棋盘格采集：图像 + 机械臂位姿
  compute_in_hand.py           # 官方眼在手上棋盘格标定
  compute_to_hand.py           # 官方眼在手外棋盘格标定
  detect_red_block.py          # 任意颜色小方块检测，保留旧入口名
  jing_hand_eye_compute.py     # Jing 小方块最小二乘手眼标定
  compute_in_hand_12data.py    # Jing 方法 12 组样例验证
  jing.py                      # 使用标定结果做相机点到基座点转换
  save_poses.py                # 机械臂位姿转齐次矩阵
  save_poses2.py               # 眼在手外位姿处理
  config.yaml                  # 棋盘格参数
  requirements.txt             # Python 依赖
  libs/                        # 日志与辅助函数
  eye_hand_data/               # 官方棋盘格采集数据目录，仓库只保留占位
  jing_data/eye_hand_data/     # Jing 小方块数据目录，仓库只保留占位
```

采集图片、运行日志、运行结果、虚拟环境和缓存文件默认不提交到仓库。

## 环境安装

建议使用 Python 3.10 或 Python 3.11：

```powershell
python -m venv venv
.\venv\Scripts\activate
pip install -r .\hand_eye_calibration\requirements.txt
```

如果需要运行 RealSense 相机检测，还要安装与相机环境匹配的 `pyrealsense2`。

## 官方棋盘格标定流程

官方流程适合使用标准棋盘格进行手眼标定。

### 1. 配置棋盘格参数

修改 [config.yaml](hand_eye_calibration/config.yaml)：

```yaml
checkerboard_args:
  XX: 11
  YY: 8
  L: 0.03
```

含义：

- `XX`：棋盘格横向内角点数量。
- `YY`：棋盘格纵向内角点数量。
- `L`：单个格子的边长，单位为米。

### 2. 采集棋盘格图像和机械臂位姿

```powershell
cd hand_eye_calibration
python collect_data.py
```

采集结果会保存到：

```text
eye_hand_data/dataYYYYMMDD.../
```

目录中包含：

- `1.jpg`, `2.jpg`, ...：采集到的棋盘格图像。
- `poses.txt`：每张图像对应的机械臂末端位姿。

注意：图片序号必须和 `poses.txt` 中的行号一一对应。

### 3. 计算眼在手上标定结果

```powershell
python compute_in_hand.py
```

输出结果为相机坐标系相对于机械臂末端坐标系的旋转矩阵和平移向量。

## Jing 小方块标定流程

Jing 方法使用一个已知真实位置的小方块替代棋盘格。它不要求方块一定是红色，也不要求方块一定是 3 cm。

核心关系为：

```text
base下的小方块坐标 ≈ base_T_end * end_T_camera * camera下的小方块坐标
```

代码要求解的是 `end_T_camera`，也就是相机坐标系到机械臂末端坐标系的变换。

## 任意颜色小方块检测

运行检测脚本：

```powershell
cd hand_eye_calibration
python detect_red_block.py
```

启动后会要求输入小方块边长，单位为厘米。也可以直接通过命令行指定：

```powershell
python detect_red_block.py --size-cm 4
```

当前 `detect_red_block.py` 的功能：

- 自动寻找近似正方形轮廓。
- 根据用户输入的真实边长和当前深度，估算理论像素边长，用于过滤误检。
- 自动学习候选方块区域的颜色，后续帧使用该颜色模型追踪。
- 支持红色、蓝色、绿色等任意明显颜色。
- 对黑色、白色等低饱和颜色，主要依赖方形轮廓和尺寸约束。
- 按 `R` 可以重置颜色模型。
- 按 `ESC` 退出检测窗口。

检测到方块后会输出方块中心在相机坐标系下的坐标：

```text
Square center: X=..., Y=..., Z=..., side=...cm
```

这些坐标就是 Jing 标定数据中的前三列。

## Jing 数据格式

Jing 标定每组数据需要 9 个数：

```text
camera_x camera_y camera_z end_x end_y end_z end_rx end_ry end_rz
```

含义：

- `camera_x camera_y camera_z`：小方块中心在相机坐标系下的位置，单位为米。
- `end_x end_y end_z`：机械臂末端在基座坐标系下的位置，单位为米。
- `end_rx end_ry end_rz`：机械臂末端姿态角，单位为弧度。

默认数据路径为：

```text
hand_eye_calibration/jing_data/eye_hand_data/dataYYYYMMDD/poses.txt
```

也支持以下文件名：

```text
jing_samples.csv
jing_samples.txt
red_block_samples.csv
red_block_samples.txt
calibration_data.csv
calibration_data.txt
poses.csv
poses.txt
```

CSV 表头可以使用：

```csv
camera_x,camera_y,camera_z,end_x,end_y,end_z,end_rx,end_ry,end_rz
```

也可以使用：

```csv
x,y,z,x1,y1,z1,rx,ry,rz
```

## 计算 Jing 标定结果

运行：

```powershell
cd hand_eye_calibration
python jing_hand_eye_compute.py --expected-obj-base 0.4,0.2,0.03
```

`--expected-obj-base` 表示小方块中心在机械臂基坐标系下的真实坐标，单位为米。

如果不传该参数，默认使用：

```text
0.4,0.2,0.03
```

也可以指定数据文件：

```powershell
python jing_hand_eye_compute.py --data-file .\jing_data\eye_hand_data\data2026030301\poses.txt --expected-obj-base 0.4,0.2,0.03
```

输出结果默认保存到：

```text
hand_eye_calibration/hand_eye_result_jing.yaml
```

结果文件包含：

- `rotation_matrix`：相机到末端的旋转矩阵。
- `translation_vector`：相机到末端的平移向量。
- `quaternion`：旋转矩阵对应的四元数。
- `expected_obj_base`：小方块在基座坐标系下的真实坐标。
- `sample_count`：参与计算的数据组数。
- `source_file`：使用的数据文件。
- `avg_error_mm`：平均误差，单位为毫米。
- `per_sample_error_mm`：每组样本的误差，单位为毫米。

## Python 调用方式

`jing_hand_eye_compute.py` 保留了和官方脚本类似的 `func()` 接口：

```python
from jing_hand_eye_compute import func

rotation_matrix, translation_vector = func(
    data_file=None,
    expected_obj_base="0.4,0.2,0.03",
)
```

返回值：

```text
rotation_matrix: (3, 3)
translation_vector: (3, 1)
```

因此其它代码如果原本接收 `compute_in_hand.func()` 的结果，也可以改成接收 `jing_hand_eye_compute.func()` 的结果。

## 两种方法的优缺点

### 官方棋盘格方法

优点：

- 理论标准，是典型的 AX=XB 手眼标定问题。
- 棋盘格角点提供较强的位姿约束。
- 不需要知道棋盘格在机械臂基坐标系下的真实位置。
- 更适合做规范化、可复现实验。

缺点：

- 需要准备棋盘格。
- 对角点检测质量比较敏感。
- 棋盘格标定得到的是几何标定结果，不一定直接反映实际抓取红块时的最终误差。

### Jing 小方块方法

优点：

- 更贴近实际抓取任务。
- 不依赖棋盘格。
- 可以直接用相机识别到的小方块中心验证“相机点转基座点”的误差。
- 方块颜色不固定，边长可由用户输入。

缺点：

- 必须知道小方块中心在机械臂基坐标系下的真实坐标。
- 只使用中心点时，对旋转的约束比棋盘格弱。
- 对深度噪声、颜色分割误差、采样姿态多样性更敏感。
- 建议采集不少于 10 组数据，并尽量覆盖不同位置和不同末端姿态。

## 后续改进方向

目前 Jing 方法主要使用小方块中心点。如果想进一步接近棋盘格方法的约束强度，可以让检测脚本输出小方块四个角点，再结合真实边长估计方块完整姿态，而不仅仅使用中心点。

这样可以把“小方块方法”从点约束扩展成位姿约束，理论上会比只用中心点更加稳定。

## 注意事项

- 机械臂末端姿态默认按 `xyz` 欧拉角处理，单位为弧度。
- 采集数据时要保证相机坐标、机械臂位姿和真实基座坐标来自同一次采样。
- Jing 方法中的 `expected_obj_base` 一定要填写小方块中心在基座坐标系下的真实坐标。
- 数据质量比算法本身更重要：深度异常、方块遮挡、机械臂位姿记录错位都会明显影响结果。
