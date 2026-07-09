# Hand-Eye Calibration Toolkit

这个仓库包含一套机械臂手眼标定实验代码，主要围绕两条路线：

1. **官方棋盘格法**：使用棋盘格图像和机械臂末端位姿，通过 OpenCV `calibrateHandEye` 求解手眼矩阵。
2. **Jing 小方块法**：使用相机识别到的方块中心 3D 坐标、机械臂末端位姿、以及方块在机械臂基坐标系下的真实坐标，通过最小二乘拟合相机到末端的变换。

`jing_hand_eye_compute.py` 和 `detect_red_block.py` 是自定义的小方块方案；`compute_in_hand.py` 是官方棋盘格流程；`compute_in_hand_12data.py` 是用 Jing 方法测出来的 12 组样例数据验证脚本。

## Directory

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

采集图片、日志、运行结果、虚拟环境和缓存文件默认不提交。

## Install

建议使用 Python 3.10 或 3.11：

```powershell
python -m venv venv
.\venv\Scripts\activate
pip install -r .\hand_eye_calibration\requirements.txt
```

如果要运行 RealSense 相机检测，还需要安装与你相机环境匹配的 `pyrealsense2`。

## Official Checkerboard Calibration

官方流程适合标准棋盘格标定：

1. 修改 [config.yaml](hand_eye_calibration/config.yaml) 中的棋盘格参数：

```yaml
checkerboard_args:
  XX: 11
  YY: 8
  L: 0.03
```

2. 采集棋盘格图像和机械臂位姿：

```powershell
cd hand_eye_calibration
python collect_data.py
```

数据会放到 `eye_hand_data/dataYYYYMMDD.../`，其中图片名和 `poses.txt` 行号需要一一对应。

3. 计算眼在手上标定：

```powershell
python compute_in_hand.py
```

输出为相机坐标系相对于机械臂末端坐标系的旋转矩阵和平移向量。

## Jing Square Calibration

Jing 方法用一个小方块替代棋盘格。它不要求固定红色，也不要求固定 3 cm。

### 1. Detect The Square

运行小方块检测：

```powershell
cd hand_eye_calibration
python detect_red_block.py
```

启动时会要求输入方块边长，单位是厘米。也可以直接传入：

```powershell
python detect_red_block.py --size-cm 4
```

现在 `detect_red_block.py` 的行为是：

- 自动寻找近似正方形轮廓；
- 根据输入的真实边长和深度估计理论像素边长，用于过滤误检；
- 自动学习候选方块区域的颜色，后续帧用该颜色模型追踪；
- 支持红、蓝、绿等任意明显颜色；低饱和的黑白方块主要依靠轮廓；
- 按 `R` 重置颜色模型，按 `ESC` 退出。

检测到方块后会打印方块中心在相机坐标系下的坐标：

```text
Square center: X=..., Y=..., Z=..., side=...cm
```

### 2. Prepare Jing Data

Jing 标定需要每组数据包含 9 个数：

```text
camera_x camera_y camera_z end_x end_y end_z end_rx end_ry end_rz
```

前三列是方块中心在相机坐标系下的位置，后六列是机械臂末端在基座坐标系下的位置和欧拉角，单位为米和弧度。

默认数据路径：

```text
hand_eye_calibration/jing_data/eye_hand_data/dataYYYYMMDD/poses.txt
```

也支持这些文件名：

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

或者：

```csv
x,y,z,x1,y1,z1,rx,ry,rz
```

### 3. Compute Jing Calibration

运行：

```powershell
cd hand_eye_calibration
python jing_hand_eye_compute.py --expected-obj-base 0.4,0.2,0.03
```

`--expected-obj-base` 是小方块中心在机械臂基座坐标系下的真实坐标，单位为米。如果不传，默认使用：

```text
0.4,0.2,0.03
```

可以指定数据文件：

```powershell
python jing_hand_eye_compute.py --data-file .\jing_data\eye_hand_data\data2026030301\poses.txt --expected-obj-base 0.4,0.2,0.03
```

输出结果默认保存到：

```text
hand_eye_calibration/hand_eye_result_jing.yaml
```

结果包含：

- `rotation_matrix`
- `translation_vector`
- `quaternion`
- `expected_obj_base`
- `sample_count`
- `source_file`
- `avg_error_mm`
- `per_sample_error_mm`

### 4. Python API

`jing_hand_eye_compute.py` 保留了和官方脚本类似的接口：

```python
from jing_hand_eye_compute import func

rotation_matrix, translation_vector = func(
    data_file=None,
    expected_obj_base="0.4,0.2,0.03",
)
```

返回：

```text
rotation_matrix: (3, 3)
translation_vector: (3, 1)
```

## Method Comparison

官方棋盘格法优点是理论标准、约束强、不需要知道目标在基座下的真实坐标；缺点是必须准备棋盘格并稳定检测角点。

Jing 小方块法优点是更贴近抓取任务，不依赖棋盘格，能直接验证相机点到基座点的误差；缺点是需要知道小方块中心在基座坐标系下的真实位置，而且只用中心点时对旋转约束较弱，对深度噪声和采样姿态多样性更敏感。

如果要进一步逼近棋盘格方法的约束强度，下一步可以让小方块检测输出四个角点，再用真实边长求方块姿态，而不是只使用中心点。

## Notes

- 机械臂末端姿态角默认按 `xyz` 欧拉角、弧度处理。
- 采集数据时要保证相机坐标、机械臂位姿和真实基座坐标对应同一次采样。
- Jing 方法建议采集至少 10 组姿态，并尽量覆盖不同位置和旋转角度。
