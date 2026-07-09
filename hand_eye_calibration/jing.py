import numpy as np
from scipy.spatial.transform import Rotation as R



# 最新手眼标定结果
# 最新手眼标定结果 (2026-03-05 16:18:54)
rotation_matrix = np.array([
    [ 0.85429295,  0.51966519, -0.01147355],
    [-0.51966369,  0.85436436,  0.00334637],
    [ 0.01154158,  0.00310361,  0.99992858]
])
translation_vector = np.array([-0.04701706, -0.09330678, 0.02553302])


def convert(x ,y ,z ,x1 ,y1 ,z1 ,rx ,ry ,rz):
    """
    我们需要将手眼标定得到旋转向量和平移向量转换为齐次变换矩阵，然后使用深度相机识别到的物体坐标（x, y, z）和
    机械臂末端的位姿（x1,y1,z1,rx,ry,rz）来计算物体相对于机械臂基座的位姿（x, y, z）

    """

    # 深度相机识别物体返回的坐标
    obj_camera_coordinates = np.array([x, y, z])

    # 机械臂末端的位姿，单位为弧度
    end_effector_pose = np.array([x1, y1, z1,
                                  rx, ry, rz])

    # 将旋转矩阵和平移向量转换为齐次变换矩阵
    T_camera_to_end_effector = np.eye(4)
    T_camera_to_end_effector[:3, :3] = rotation_matrix
    T_camera_to_end_effector[:3, 3] = translation_vector

    # 机械臂末端的位姿转换为齐次变换矩阵
    position = end_effector_pose[:3]
    orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()

    T_base_to_end_effector = np.eye(4)
    T_base_to_end_effector[:3, :3] = orientation
    T_base_to_end_effector[:3, 3] = position

    # 计算物体相对于机械臂基座的位姿
    obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])  # 将物体坐标转换为齐次坐标

    obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(obj_camera_coordinates_homo)

    obj_base_coordinates_homo = T_base_to_end_effector.dot(obj_end_effector_coordinates_homo)

    obj_base_coordinates = list(obj_base_coordinates_homo[:3])  # 从齐次坐标中提取物体的x, y, z坐标

    return obj_base_coordinates


def main():
    """主程序：输入相机观测坐标，输出基座标位置"""
    print("手眼标定坐标转换程序")
    print("=" * 50)

    try:
        # 输入相机观测的物体坐标
        x = float(input("物体在相机坐标系的 x 坐标 (米): "))
        y = float(input("物体在相机坐标系的 y 坐标 (米): "))
        z = float(input("物体在相机坐标系的 z 坐标 (米): "))

        # 输入机械臂末端位姿
        x1 = float(input("机械臂末端 x 坐标 (米): "))
        y1 = float(input("机械臂末端 y 坐标 (米): "))
        z1 = float(input("机械臂末端 z 坐标 (米): "))

        # 输入旋转角度（默认弧度）
        rx = float(input("机械臂末端绕 x 轴旋转角度 (弧度): "))
        ry = float(input("机械臂末端绕 y 轴旋转角度 (弧度): "))
        rz = float(input("机械臂末端绕 z 轴旋转角度 (弧度): "))

        # 执行转换
        result = convert(x, y, z, x1, y1, z1, rx, ry, rz)

        # 输出结果
        print("\n" + "=" * 50)
        print("转换结果：")
        print(f"物体在机械臂基坐标系的坐标：")
        print(f"x = {result[0]:.6f} 米")
        print(f"y = {result[1]:.6f} 米")
        print(f"z = {result[2]:.6f} 米")
        print(f"\n（毫米表示）：")
        print(f"x = {result[0] * 1000:.2f} mm")
        print(f"y = {result[1] * 1000:.2f} mm")
        print(f"z = {result[2] * 1000:.2f} mm")

    except ValueError:
        print("错误：请输入有效的数字！")
    except Exception as e:
        print(f"转换过程中发生错误：{e}")


if __name__ == "__main__":
    main()