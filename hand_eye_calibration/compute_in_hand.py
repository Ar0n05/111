# coding=utf-8

"""
眼在手上 用采集到的图片信息和机械臂位姿信息计算 相机坐标系相对于机械臂末端坐标系的 旋转矩阵和平移向量
A2^{-1}*A1*X=X*B2*B1^{−1}
"""

import os
import logging
import yaml
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from libs.auxiliary import find_latest_data_folder
from libs.log_setting import CommonLog

from save_poses import poses_main

np.set_printoptions(precision=8,suppress=True)

logger_ = logging.getLogger(__name__)
logger_ = CommonLog(logger_)


current_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"eye_hand_data")

images_path = os.path.join("eye_hand_data",find_latest_data_folder(current_path))
file_path = os.path.join(images_path,"poses.txt")  #采集标定板图片时对应的机械臂末端的位姿 从 第一行到最后一行 需要和采集的标定板的图片顺序进行对应
print(images_path)

with open("config.yaml", 'r', encoding='utf-8') as file:
    data = yaml.safe_load(file)

XX = data.get("checkerboard_args").get("XX") #标定板的中长度对应的角点的个数
YY = data.get("checkerboard_args").get("YY") #标定板的中宽度对应的角点的个数
L = data.get("checkerboard_args").get("L")   #标定板一格的长度  单位为米


# ==================== 加载相机内参 ====================
def load_camera_calib():
    """加载相机内参和畸变系数"""
    try:
        # 尝试多个可能的路径
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
            raise FileNotFoundError(f"找不到相机内参文件，已尝试: {possible_paths}")
        
        logger_.info(f"正在加载内参文件: {calib_path}")
        
        # 从 YAML 文件加载内参
        with open(calib_path, 'r', encoding='utf-8') as f:
            calib_data = yaml.safe_load(f)
        # 适配不同的YAML格式
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

# 加载内参
mtx, dist = load_camera_calib()


def func():

    path = os.path.dirname(__file__)

    # 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
    criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

    # 获取标定板角点的位置
    objp = np.zeros((XX * YY, 3), np.float32)
    objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)     # 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
    objp = L*objp

    obj_points = []     # 存储3D点
    img_points = []     # 存储2D点

    images_num = [f for f in os.listdir(images_path) if f.endswith('.jpg')]

    for i in range(1, len(images_num) + 1):   #标定好的图片在images_path路径下，从0.jpg到x.jpg

        image_file = os.path.join(images_path,f"{i}.jpg")

        if os.path.exists(image_file):

            logger_.info(f'读 {image_file}')

            img = cv2.imread(image_file)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            size = gray.shape[::-1]
            ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)

            if ret:
                # ==================== 使用加载的内参校正畸变 ====================
                if mtx is not None and dist is not None:
                    # 转换角点格式为适合 undistortPoints 的输入
                    corners_reshaped = corners.reshape(-1, 1, 2)
                    # 校正畸变
                    corners_undistorted = cv2.undistortPoints(corners_reshaped, mtx, dist, P=mtx)
                    corners = corners_undistorted.reshape(-1, 1, 2)

                obj_points.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 在原角点的基础上寻找亚像素角点
                if [corners2]:
                    img_points.append(corners2)
                else:
                    img_points.append(corners)

    N = len(img_points)

    # ==================== 使用加载的内参进行标定 ====================
    if mtx is not None and dist is not None:
        # 使用已知内参计算标定板位姿
        rvecs = []
        tvecs = []
        for objp, imgp in zip(obj_points, img_points):
            ret, rvec, tvec = cv2.solvePnP(objp, imgp, mtx, dist)
            if ret:
                rvecs.append(rvec)
                tvecs.append(tvec)
        logger_.info("使用预加载的内参计算标定板位姿")
    else:
        # 若内参加载失败，使用默认方法重新标定
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)
        logger_.info("内参加载失败，使用默认方法重新标定")

    # logger_.info(f"内参矩阵:\n:{mtx}" ) # 内参数矩阵
    # logger_.info(f"畸变系数:\n:{dist}")  # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)

    print("-----------------------------------------------------")

    poses_main(file_path)
    # 机器人末端在基座标系下的位姿

    csv_file = os.path.join(path,"RobotToolPose.csv")
    tool_pose = np.loadtxt(csv_file,delimiter=',')

    R_tool = []
    t_tool = []

    for i in range(int(N)):

        R_tool.append(tool_pose[0:3,4*i:4*i+3])
        t_tool.append(tool_pose[0:3,4*i+3])

    R, t = cv2.calibrateHandEye(R_tool, t_tool, rvecs, tvecs, cv2.CALIB_HAND_EYE_TSAI)

    return R,t

if __name__ == '__main__':

    # 旋转矩阵
    rotation_matrix, translation_vector = func()

    # 将旋转矩阵转换为四元数
    rotation = R.from_matrix(rotation_matrix)
    quaternion = rotation.as_quat()
    x, y, z = translation_vector.flatten()

    logger_.info(f"旋转矩阵是:\n {            rotation_matrix}")

    logger_.info(f"平移向量是:\n {            translation_vector}")

    logger_.info(f"四元数是：\n {             quaternion}")