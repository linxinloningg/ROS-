import pybullet
import os
import pybullet_data
from math import pi
from fun import create_gui_joint_sliders, read_gui_sliders, set_joint_angle, get_joint_angle
from fun import Draw
from transform import Posture

# 工具默认位置
WRIST_DEFAULT_POSITION = [600, 0, 400]
# 工具默认欧垃角
WRIST_DEFAULT_EULER_ANGLE = [pi, 0, 0]
ARM_DEAFULT_JOINT_ANGLE = [0, -pi / 2, pi / 2, 0, 0, 0]

# URDF模型
ROBOT_URDF_PATH = "./model/urdf/ur5e.urdf"
control_joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                  "wrist_2_joint", "wrist_3_joint"]

if __name__ == "__main__":
    # 连接物理引擎
    _ = pybullet.connect(pybullet.GUI)

    # 渲染逻辑
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)

    # 设置环境重力加速度
    pybullet.setGravity(0, 0, -10)
    pybullet.setRealTimeSimulation(True)
    # 取消两侧控件
    # pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
    # 取消使用集显来渲染
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_TINY_RENDERER, 0)

    plane_id = pybullet.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

    flags = pybullet.URDF_USE_SELF_COLLISION  # 自碰撞检测标志位
    robot = pybullet.loadURDF(ROBOT_URDF_PATH, [0, 0, 0], [0, 0, 0, 1], flags=flags)
    # 画出GUI滑动条
    joint_sliders = create_gui_joint_sliders(pybullet, ARM_DEAFULT_JOINT_ANGLE)

    joint_info = dict()
    joint_type_list = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
    for i in range(pybullet.getNumJoints(robot)):
        info_dict = dict()

        info = pybullet.getJointInfo(robot, i)
        name = info[1].decode("utf-8")
        info_dict.setdefault("id", info[0])
        info_dict.setdefault("name", name)
        info_dict.setdefault("type", joint_type_list[info[2]])
        info_dict.setdefault("lowerLimit", info[8])
        info_dict.setdefault("upperLimit", info[9])
        info_dict.setdefault("maxForce", info[10])
        info_dict.setdefault("maxVelocity", info[11])
        controllable = True if name in control_joints else False
        info_dict.setdefault("controllable", controllable)
        if info_dict["type"] == "REVOLUTE":
            # 控制单个旋转关节的速度
            # 设置机器人关节角度， 电机随着仿真逐步运动到对应的位置
            # 控制模式: VELOCITY_CONTROL 速度控制
            # 目标速度:  0.0
            pybullet.setJointMotorControl2(robot, info_dict["id"], pybullet.VELOCITY_CONTROL, targetVelocity=0,
                                           force=0)
        joint_info.setdefault(name, info_dict)

    # 目标坐标系
    painter = Draw(pybullet, pose=None, name="target")
    # 绘制初始化
    painter.init_pose_axis_draw()
    painter.init_pose_data_draw()

    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
    while True:
        try:
            # 不断读取GUI滑动条数据
            angles = read_gui_sliders(pybullet, joint_sliders)

            # 设置末端节点位姿并控制机械臂运动
            return_posture = set_joint_angle(pybullet, robot, joint_info, control_joints, joint_angles=angles)

            target_pose = Posture()
            target_pose.set_position(*return_posture[:3])
            target_pose.set_euler_angle(*return_posture[3:])

            # 绘制目标位姿坐标系
            painter.pose_axis_draw(target_pose)
            # 绘制目标位姿数据
            painter.pose_data_draw(target_pose)

            # 获取当前腕关节位姿
            cur_angles = get_joint_angle(pybullet, robot)
        except Exception as e:
            print(e)
            continue
