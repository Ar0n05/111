# coding=utf-8
from robotic_arm_package.robotic_arm import *
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

# # 相机坐标系到机械臂末端坐标系的旋转矩阵和平移向量（手眼标定得到）
# rotation_matrix = np.array([[0.91186673 , 0.40971645 ,0.02512949],
#                             [-0.41042577, 0.9089702, 0.07296483],
#                             [0.00705293, -0.07684799, 0.99701787]])
# # translation_vector = np.array([-0.03042881, -0.09776948, 0.01323594])
# translation_vector = np.array([-30.42881, -97.76948, 13.23594])
# # 平移向量（单位从米变成毫米）


# 相机坐标系到机械臂末端坐标系的旋转矩阵和平移向量（手眼标定得到）
# rotation_matrix = np.array([[ 0.86662555, 0.49874631, -0.01456954],
#                             [-0.49889363, 0.8666153,  -0.00911429],
#                             [ 0.00808047, 0.01516733,  0.99985232]])
# translation_vector = np.array([-0.04099113, -0.09466831, 0.03420196])
# # translation_vector = np.array([-40.99113, -94.66831, 34.20196])
# 平移向量（单位从米变成毫米）

rotation_matrix = np.array([
    [0.88903804, 0.44649732, -0.10124970],
    [-0.45405134, 0.88822990, -0.06989292],
    [0.05872601, 0.10811002, 0.99240288]
])
translation_vector = np.array([-0.38935312, -0.13172878, -0.01914676])

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

#   抓红色方块
def demo90(robot):
    ret = robot.Movej_Cmd([2.8, -9.462, -64.373, 1.74, -104.685, 120.745], 20, 0)
    # 移动到机械臂初始状态
    if ret != 0:
        print("到达抓取位置失败：" + str(ret))
        sys.exit()

    # #   张开夹爪，抓取位置
    # robot.Set_Gripper_Release(500)
    # if ret != 0:
    #     print("张开夹爪失败：" + str(ret))
    #     sys.exit()
    #
    # #   抓取
    # ret = robot.Set_Gripper_Pick_On(500, 500)
    # if ret != 0:
    #     print("抓取失败：" + str(ret))
    #     sys.exit()




# 位置示教例程
def demo4(robot):
    # 初始位置
    joint = [0, -20, -70, 0, -90, 0]
    zero = [0, 0, 0, 0, 0, 0]

    robot.Movej_Cmd(joint, 20, 0)

    # 切换示教坐标系为基坐标系
    robot.Set_Teach_Frame(0)
    #   位置示教
    ret = robot.Pos_Teach_Cmd(2, 1, 10)
    time.sleep(2)
    if ret != 0:
        print("Z轴正方向示教失败：" + str(ret))
        sys.exit()

    ret = robot.Teach_Stop_Cmd()
    if ret != 0:
        print("停止示教失败：" + str(ret))
        sys.exit()

    # 切换示教坐标系为工具坐标系
    robot.Set_Teach_Frame(1)

    #   位置示教
    ret = robot.Pos_Teach_Cmd(2, 1, 20)
    time.sleep(2)
    if ret != 0:
        print("Z轴正方向示教失败：" + str(ret))
        sys.exit()

    ret = robot.Teach_Stop_Cmd()
    if ret != 0:
        print("停止示教失败：" + str(ret))
        sys.exit()

    print("Z轴位置示教成功")


if __name__ == "__main__":
    def mcallback(data):
        print("MCallback MCallback MCallback")
        # 判断接口类型
        if data.codeKey == MOVEJ_CANFD_CB:  # 角度透传
            print("透传结果:", data.errCode)
            print("当前角度:", data.joint[0], data.joint[1], data.joint[2], data.joint[3], data.joint[4], data.joint[5])
        elif data.codeKey == MOVEP_CANFD_CB:  # 位姿透传
            print("透传结果:", data.errCode)
            print("当前角度:", data.joint[0], data.joint[1], data.joint[2], data.joint[3], data.joint[4], data.joint[5])
            print("当前位姿:", data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.euler.rx,
                  data.pose.euler.ry, data.pose.euler.rz)
        elif data.codeKey == FORCE_POSITION_MOVE_CB:  # 力位混合透传
            print("透传结果:", data.errCode)
            print("当前力度：", data.nforce)


    # 连接机械臂，注册回调函数
    callback = CANFD_Callback(mcallback)
    robot = Arm(RM65, "192.168.1.18", callback)

    # API版本信息
    print(robot.API_Version())

    # 运行示例
    # demo90(robot)

    # obj_b_coor=convert(0.0208, 0.035, 0.359 , 246.982, 16.340, 407.579, 3.148, 0.038, -2.050)
    # obj_b_coor = convert(35, 20.8, 359, 246.982, 16.340, 407.579, 3.148, 0.038, -2.050)
    # obj_b_coor = convert(221, 290, 332, 217.713, -1.69, 365.892, -3.027, 0.074, -2.058)
    obj_b_coor = convert(0.513, 0.259, 0.357, 0.147383, 0.004810, 0.352453, -2.811, 0.275, -1.714)
    print(obj_b_coor)

    # 断开连接
    robot.RM_API_UnInit()
    robot.Arm_Socket_Close()
