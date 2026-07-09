# coding=utf-8

"""
眼在手上 - 使用12组特定数据计算手眼标定
基于用户提供的12组对应数据
"""

import os
import logging
import yaml
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as SciR
from scipy.optimize import least_squares

from libs.log_setting import CommonLog

np.set_printoptions(precision=8, suppress=True)

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)


# ==================== 加载相机内参 ====================
def load_camera_calib():
    """加载相机内参和畸变系数"""
    try:
        possible_paths = [
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_calibration.yaml"),
            "camera_calibration.yaml",
            os.path.join(os.getcwd(), "camera_calibration.yaml"),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "hand_eye_calibration-main", "camera_calibration.yaml"),
            r"C:\Users\Dell\Desktop\petest\hand_eye_calibration-main\camera_calibration.yaml",
        ]
        
        calib_path = None
        for path in possible_paths:
            if os.path.exists(path):
                calib_path = path
                break
        
        if calib_path is None:
            raise FileNotFoundError(f"找不到相机内参文件")
        
        logger_.info(f"正在加载内参文件: {calib_path}")
        
        with open(calib_path, 'r', encoding='utf-8') as f:
            calib_data = yaml.safe_load(f)
        if 'mtx' in calib_data:
            mtx = np.array(calib_data['mtx'])
            dist = np.array(calib_data['dist'])
        elif 'camera_matrix' in calib_data:
            mtx = np.array(calib_data['camera_matrix'])
            dist = np.array(calib_data['distortion_coefficients'])
        else:
            raise ValueError("内参文件格式错误")
        logger_.info("成功加载相机内参")
        return mtx, dist
    except Exception as e:
        logger_.error(f"加载内参失败: {e}")
        return None, None


# ==================== 用户提供的12组数据 ====================
# 格式: [相机x, 相机y, 相机z, 机械臂x, 机械臂y, 机械臂z, rx, ry, rz]
# 期望的基座标: [0.4, 0.2, 0.03]
calibration_data = [
    [0.169, 0.262, 0.448, 0.177472, -0.079656, 0.378485, 3.118, 0.428, -2.309],
    [0.322, 0.239, 0.431, 0.190201, 0.013183, 0.393212, -3.008, 0.494, -1.835],
    [0.4, 0.217, 0.405, 0.157929, 0.077679, 0.387674, -2.841, 0.194, -1.262],
    [0.383, 0.126, 0.503, 0.062876, 0.028444, 0.477567, -2.910, 0.056, -1.250],
    [0.15, 0.23, 0.320, 0.215687, -0.028003, 0.268973, 3.042, 0.432, -2.265],
    [0.146, 0.352, 0.391, 0.182260, -0.106315, 0.253505, 2.712, 0.485, -2.921],
    [0.143, 0.334, 0.454, 0.169218, -0.124849, 0.347778, 2.993, 0.486, -2.63],
    [0.352, 0.202, 0.359, 0.183035, 0.095762, 0.351253, -2.821, -0.174, -0.993],
    [0.384, 0.101, 0.518, 0.017567, -0.009179, 0.465331, -2.850, -0.084, -1.114],
    [0.408, 0.175, 0.308, 0.172792, 0.079650, 0.294122, 3.148, 0.609, -1.889],
    [0.458, 0.224, 0.332, 0.133755, 0.163986, 0.317839, 2.933, -0.575, -0.112],
    [0.183, 0.176, 0.299, 0.229902, -0.001832, 0.278856, 3.130, 0.345, -1.925]
]

expected_obj_base = np.array([0.4, 0.2, 0.03])


def euler_to_rotation_matrix(rx, ry, rz):
    """将欧拉角转换为旋转矩阵 (ZYX顺序)"""
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])

    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])

    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])

    return Rz @ Ry @ Rx


def optimize_hand_eye():
    """使用12组数据优化手眼变换矩阵"""
    
    # 预处理数据
    obj_camera_list = []  # 相机坐标系下的物体坐标
    T_base_to_end_list = []  # 机械臂末端到基座的变换矩阵
    
    for item in calibration_data:
        x, y, z, x1, y1, z1, rx, ry, rz = item
        
        # 相机坐标系下的物体坐标（齐次）
        obj_camera = np.array([x, y, z, 1])
        obj_camera_list.append(obj_camera)
        
        # 计算机械臂末端到基座的变换矩阵 T_base_to_end
        T_base_to_end = np.eye(4)
        T_base_to_end[:3, :3] = euler_to_rotation_matrix(rx, ry, rz)
        T_base_to_end[:3, 3] = [x1, y1, z1]
        T_base_to_end_list.append(T_base_to_end)
    
    # 误差函数
    def error_function(params):
        rx_cam, ry_cam, rz_cam, tx, ty, tz = params
        
        # 构建相机到末端的变换矩阵
        T_camera_to_end = np.eye(4)
        T_camera_to_end[:3, :3] = euler_to_rotation_matrix(rx_cam, ry_cam, rz_cam)
        T_camera_to_end[:3, 3] = [tx, ty, tz]
        
        # 计算每个数据点的误差
        errors = []
        for i in range(len(obj_camera_list)):
            obj_camera = obj_camera_list[i]
            T_base_to_end = T_base_to_end_list[i]
            
            # 坐标转换：相机 -> 末端 -> 基座
            obj_end = T_camera_to_end @ obj_camera
            obj_base = T_base_to_end @ obj_end
            
            # 计算与期望坐标的误差
            error = obj_base[:3] - expected_obj_base
            errors.extend(error)
        
        return np.array(errors)
    
    # 初始猜测（单位矩阵）
    initial_params = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # 优化
    result = least_squares(error_function, initial_params, method='lm')
    
    # 提取优化后的参数
    optimized_euler = result.x[:3]
    optimized_translation = result.x[3:]
    optimized_rotation = euler_to_rotation_matrix(*optimized_euler)
    
    # 计算总误差
    total_error = np.linalg.norm(error_function(result.x))
    avg_error = total_error / len(calibration_data)
    
    return optimized_rotation, optimized_translation, total_error, avg_error


def verify_results(rotation_matrix, translation_vector):
    """验证优化结果"""
    print("\n验证每个数据点的转换结果:")
    print("=" * 80)
    print(f"{'数据点':<8} {'计算基座标':<30} {'期望基座标':<20} {'误差(mm)':<10}")
    print("=" * 80)
    
    # 构建变换矩阵
    T_camera_to_end = np.eye(4)
    T_camera_to_end[:3, :3] = rotation_matrix
    T_camera_to_end[:3, 3] = translation_vector
    
    # 验证每个数据点
    errors = []
    for i, item in enumerate(calibration_data):
        x, y, z, x1, y1, z1, rx, ry, rz = item
        
        # 相机坐标系下的物体坐标（齐次）
        obj_camera = np.array([x, y, z, 1])
        
        # 计算机械臂末端到基座的变换矩阵
        T_base_to_end = np.eye(4)
        T_base_to_end[:3, :3] = euler_to_rotation_matrix(rx, ry, rz)
        T_base_to_end[:3, 3] = [x1, y1, z1]
        
        # 坐标转换：相机 -> 末端 -> 基座
        obj_end = T_camera_to_end @ obj_camera
        obj_base = T_base_to_end @ obj_end
        
        # 计算误差
        error = np.linalg.norm(obj_base[:3] - expected_obj_base)
        errors.append(error)
        
        # 打印结果
        print(f"{i+1:<8} [{obj_base[0]:.4f}, {obj_base[1]:.4f}, {obj_base[2]:.4f}]  [{expected_obj_base[0]:.4f}, {expected_obj_base[1]:.4f}, {expected_obj_base[2]:.4f}]  {error * 1000:.2f}")
    
    print("=" * 80)
    
    # 计算平均误差
    avg_error = np.mean(errors)
    print(f"\n平均误差: {avg_error} 米")
    print(f"平均误差: {avg_error * 1000:.2f} 毫米")


def main():
    """主程序"""
    print("=" * 80)
    print("手眼标定 - 使用12组特定数据优化")
    print("=" * 80)
    print(f"数据组数: {len(calibration_data)}")
    print(f"期望基座标: {expected_obj_base}")
    print("=" * 80)
    
    # 优化变换矩阵
    rotation_matrix, translation_vector, total_error, avg_error = optimize_hand_eye()
    
    # 打印优化结果
    print("\n优化结果：")
    print(f"总误差: {total_error:.6f}")
    print(f"平均误差: {avg_error:.6f} 米 ({avg_error * 1000:.2f} 毫米)")
    
    # 验证结果
    verify_results(rotation_matrix, translation_vector)
    
    # 打印最终的变换参数
    print("\n" + "=" * 80)
    print("最终的相机到末端的变换参数:")
    print("=" * 80)
    print("rotation_matrix = np.array([")
    print(f"    [{rotation_matrix[0, 0]:.8f}, {rotation_matrix[0, 1]:.8f}, {rotation_matrix[0, 2]:.8f}],")
    print(f"    [{rotation_matrix[1, 0]:.8f}, {rotation_matrix[1, 1]:.8f}, {rotation_matrix[1, 2]:.8f}],")
    print(f"    [{rotation_matrix[2, 0]:.8f}, {rotation_matrix[2, 1]:.8f}, {rotation_matrix[2, 2]:.8f}]")
    print("])")
    print(f"translation_vector = np.array([{translation_vector[0]:.8f}, {translation_vector[1]:.8f}, {translation_vector[2]:.8f}])")
    
    # 与期望结果对比
    expected_R = np.array([
        [0.88903804, 0.44649732, -0.10124970],
        [-0.45405134, 0.88822990, -0.06989292],
        [0.05872601, 0.10811002, 0.99240288]
    ])
    expected_t = np.array([-0.38935312, -0.13172878, -0.01914676])
    
    print("\n" + "=" * 80)
    print("与之前优化结果的对比:")
    print("=" * 80)
    R_error = np.linalg.norm(rotation_matrix - expected_R, 'fro')
    t_error = np.linalg.norm(translation_vector - expected_t)
    print(f"旋转矩阵误差 (Frobenius范数): {R_error:.6f}")
    print(f"平移向量误差 (L2范数): {t_error:.6f} 米 ({t_error * 1000:.2f} 毫米)")
    
    # 保存结果到文件
    result_file = os.path.join(os.path.dirname(__file__), "hand_eye_result_12data.yaml")
    result_data = {
        'rotation_matrix': rotation_matrix.tolist(),
        'translation_vector': translation_vector.tolist(),
        'expected_obj_base': expected_obj_base.tolist(),
        'avg_error_mm': float(avg_error * 1000)
    }
    with open(result_file, 'w', encoding='utf-8') as f:
        yaml.dump(result_data, f, default_flow_style=False)
    print(f"\n结果已保存到: {result_file}")


if __name__ == '__main__':
    main()
