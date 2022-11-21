from transform import Transform, Posture
from numpy import inf, abs, float32, dot, degrees, eye
from kinematic import UR5Kinematic
from math import pi

# UR5运动学解析器
UR5Kinematic = UR5Kinematic()


def create_gui_posture_sliders(pybullet, WRIST_DEFAULT_POSITION, WRIST_DEFAULT_EULER_ANGLE):
    """滑动条 GUI """
    sliders = []
    # pose = arm.get_wrist_pose()
    # print(f"当前机械臂位姿 ：  {pose}")
    x0, y0, z0 = WRIST_DEFAULT_POSITION  # pose.get_position()
    roll0, pitch0, yaw0 = WRIST_DEFAULT_EULER_ANGLE  # pose.get_euler_angle()
    sliders.append(pybullet.addUserDebugParameter("X", -600, 600, x0))
    sliders.append(pybullet.addUserDebugParameter("Y", -600, 600, y0))
    sliders.append(pybullet.addUserDebugParameter("Z", 0, 600, z0))
    sliders.append(pybullet.addUserDebugParameter("Rx", -pi, pi, roll0))
    sliders.append(pybullet.addUserDebugParameter("Ry", -pi, pi, pitch0))
    sliders.append(pybullet.addUserDebugParameter("Rz", -pi, pi, yaw0))
    return sliders


def create_gui_joint_sliders(pybullet, ARM_DEAFULT_JOINT_ANGLE):
    sliders = [pybullet.addUserDebugParameter("joint1", -pi, pi, ARM_DEAFULT_JOINT_ANGLE[0]),
               pybullet.addUserDebugParameter("joint2", -pi, pi, ARM_DEAFULT_JOINT_ANGLE[1]),
               pybullet.addUserDebugParameter("joint3", -pi, pi, ARM_DEAFULT_JOINT_ANGLE[2]),
               pybullet.addUserDebugParameter("joint4", -pi, pi, ARM_DEAFULT_JOINT_ANGLE[3]),
               pybullet.addUserDebugParameter("joint5", -pi, pi, ARM_DEAFULT_JOINT_ANGLE[4]),
               pybullet.addUserDebugParameter("joint6", -pi, pi, ARM_DEAFULT_JOINT_ANGLE[5])]
    return sliders


def read_gui_sliders(pybullet, sliders):
    """GUI滑动条 数据读取"""
    data = list()
    for slider in sliders:
        data.append(pybullet.readUserDebugParameter(slider))
    return data


def get_joint_angle(pybullet, robot_id):
    """获取关节角度"""
    # 获取关节状态
    j = pybullet.getJointStates(robot_id, [1, 2, 3, 4, 5, 6])
    joints = [i[0] for i in j]
    # 关节5角度取反
    joints[-2] *= -1
    # print(joints)
    return joints


def set_joint_angle(pybullet, robot_id, joint_info, control_joints, joint_angles):
    """设置关节角度"""
    # poses = []
    indexes = []
    forces = []
    # 关节5角度区反
    joint_angles[-2] *= -1
    for i, name in enumerate(control_joints):
        info_data = joint_info[name]
        # poses.append(joint_angles[i])
        indexes.append(info_data["id"])
        forces.append(info_data["maxForce"])
    # 控制多个旋转关节的角度
    # 位置控制 制定速度为0，位置为关节目标角度
    # 位置增益: Kp
    # 正向运动学
    pose = UR5Kinematic.forward_kinematic(joint_angles)
    x, y, z = pose.get_position()
    rx, ry, rz = pose.get_euler_angle()
    pybullet.setJointMotorControlArray(
        robot_id, indexes,
        pybullet.POSITION_CONTROL,
        targetPositions=joint_angles,
        targetVelocities=[0] * len(joint_angles),
        positionGains=[0.06] * len(joint_angles), forces=forces)

    # return x, y, z, degrees(rx), degrees(ry), degrees(rz)
    return x, y, z, rx, ry, rz


def get_wrist_pose(pybullet, robot_id, end_effector_index):
    """读取腕关节位姿"""
    linkstate = pybullet.getLinkState(robot_id, end_effector_index, computeForwardKinematics=True)
    position, orientation = linkstate[0], linkstate[1]
    roll, pitch, yaw = pybullet.getEulerFromQuaternion(orientation)
    pose = Posture()
    # 位置单位m转换为mm
    pose.set_position(*[v * 1000.0 for v in position])
    # 绕X轴旋转90度

    r07 = Transform.euler2rmat(roll=roll, pitch=pitch, yaw=yaw)
    r76 = Transform.rotate_z(pi)[:3, :3]
    r06 = dot(r07, r76)
    pose.set_rotation_matrix(r06)
    # pose.set_euler_angle(*euler_angle)
    return pose


def set_wrist_pose(pybullet, robot_id, joint_info, control_joints, position=None, euler_angle=None,
                   last_joint_angle=None):
    """设置腕关节位姿"""
    # 获取当前位姿
    cur_pose = get_wrist_pose(pybullet, robot_id, end_effector_index=8)

    if last_joint_angle is None:
        # 获取当前关节角度
        last_joint_angle = get_joint_angle(pybullet, robot_id)

    # 参数补全
    if position is None:
        position = cur_pose.get_position()
    if euler_angle is None:
        euler_angle = cur_pose.get_euler_angle()

    # 目标位姿
    target_pose = Posture()
    target_pose.set_position(*position)
    target_pose.set_euler_angle(*euler_angle)
    # 逆向运动学
    candi_joint_angle_list = UR5Kinematic.inverse_kinematic_constraint(target_pose, last_joint_angle=last_joint_angle)
    if len(candi_joint_angle_list) == 0:
        candi_joint_angle_list = UR5Kinematic.inverse_kinematic(target_pose,
                                                                last_joint_angle=last_joint_angle)
        if len(candi_joint_angle_list) == 0:
            return False
    # 寻找一个最接近现在状态的解
    # 留意以下 -pi到pi之间的跳变
    distance_min = inf
    best_joint_angle = None

    for joint_angle in candi_joint_angle_list:
        joint_angle_diff = [0 for _ in range(6)]
        for i in range(6):
            angle_a = joint_angle[i]
            angle_b = last_joint_angle[i]

            angle_diff1 = abs(angle_a - angle_b)
            angle_diff2 = abs(2 * pi + angle_a - angle_b)
            angle_diff3 = abs(-2 * pi + angle_a - angle_b)

            if angle_diff2 < angle_diff1 and angle_a < 0:
                # 从pi到-pi的过渡
                joint_angle_diff[i] = angle_diff2
                joint_angle[i] += 2 * pi
            elif angle_diff3 < angle_diff1 and angle_a > 0:
                # 从-pi到pi的过渡
                joint_angle_diff[i] = angle_diff3
                joint_angle[i] -= 2 * pi
            else:
                joint_angle_diff[i] = angle_diff1

        joint_angle_diff = float32(joint_angle_diff)

        distance = sum(joint_angle_diff)  # dot(joint_angle_diff, float32(nn_weight).reshape((-1, 1)))
        if distance < distance_min:
            distance_min = distance
            best_joint_angle = joint_angle

    # 设置关节目标角度
    set_joint_angle(pybullet, robot_id, joint_info, control_joints, best_joint_angle)

    return [degrees(angle) for angle in best_joint_angle]


class Draw:
    """绘制坐标系"""

    def __init__(self, pybullet, pose=None, axis_len=0.1, line_width=5, duration=0, name="", name_color=[0, 0, 0],
                 name_size=2.0) -> None:
        self.pybullet = pybullet
        # 坐标系名称
        self.name = name
        # 名字颜色
        self.name_color = name_color
        # 名字尺寸
        self.name_size = name_size
        # 位姿 单位mm
        self.pose = pose
        # 坐标轴长度 单位m
        self.axis_len = axis_len
        # 坐标轴线宽
        self.line_width = line_width
        # 坐标系展示时间
        self.duration = duration

    def init_pose_axis_draw(self):
        """绘制坐标系"""
        pose = self.pose
        if pose is None:
            t = eye(4)
        else:
            t = pose.get_transform_matrix()
        # 绘制目标位姿
        # mm 转换为m
        origin = t[:3, 3] * 0.001
        r = t[:3, :3]
        axis_x = origin + self.axis_len * r[:, 0]
        axis_y = origin + self.axis_len * r[:, 1]
        axis_z = origin + self.axis_len * r[:, 2]
        # 绘制X轴
        self.axis_x_id = self.pybullet.addUserDebugLine(
            lineFromXYZ=list(origin.reshape(-1)),
            lineToXYZ=list(axis_x.reshape(-1)),
            lineColorRGB=[1, 0, 0],
            lineWidth=self.line_width,
            lifeTime=self.duration)
        # 绘制Y轴
        self.axis_y_id = self.pybullet.addUserDebugLine(
            lineFromXYZ=origin.reshape(-1),
            lineToXYZ=axis_y.reshape(-1),
            lineColorRGB=[0, 0, 0],
            lineWidth=self.line_width,
            lifeTime=self.duration)
        # 绘制Z轴
        self.axis_z_id = self.pybullet.addUserDebugLine(
            lineFromXYZ=origin.reshape(-1),
            lineToXYZ=axis_z.reshape(-1),
            lineColorRGB=[0, 0, 1],
            lineWidth=self.line_width,
            lifeTime=self.duration)

        # 绘制名字
        tx, ty, tz = origin
        tx += 0.02
        ty += 0.02
        tz -= 0.02
        self.name_id = self.pybullet.addUserDebugText(
            text=self.name,
            textPosition=[tx, ty, tz],
            textColorRGB=self.name_color,
            textSize=self.name_size
        )

    def pose_axis_draw(self, pose):
        """更新坐标系位姿"""
        self.pose = pose
        if pose is None:
            t = eye(4)
        else:
            t = pose.get_transform_matrix()
        # 绘制目标位姿
        # mm 转换为m
        origin = t[:3, 3] * 0.001
        r = t[:3, :3]
        axis_x = origin + self.axis_len * r[:, 0]
        axis_y = origin + self.axis_len * r[:, 1]
        axis_z = origin + self.axis_len * r[:, 2]
        # 绘制X轴
        self.pybullet.addUserDebugLine(
            lineFromXYZ=list(origin.reshape(-1)),
            lineToXYZ=list(axis_x.reshape(-1)),
            lineColorRGB=[1, 0, 0],
            lineWidth=self.line_width,
            lifeTime=self.duration,
            replaceItemUniqueId=self.axis_x_id)
        # 绘制Y轴
        self.pybullet.addUserDebugLine(
            lineFromXYZ=origin.reshape(-1),
            lineToXYZ=axis_y.reshape(-1),
            lineColorRGB=[0, 0, 0],
            lineWidth=self.line_width,
            lifeTime=self.duration,
            replaceItemUniqueId=self.axis_y_id)
        # 绘制Z轴
        self.pybullet.addUserDebugLine(
            lineFromXYZ=origin.reshape(-1),
            lineToXYZ=axis_z.reshape(-1),
            lineColorRGB=[0, 0, 1],
            lineWidth=self.line_width,
            lifeTime=self.duration,
            replaceItemUniqueId=self.axis_z_id)

        # 绘制名字
        tx, ty, tz = origin
        tx += 0.02
        ty += 0.02
        tz -= 0.02
        self.pybullet.addUserDebugText(
            text=self.name,
            textPosition=[tx, ty, tz],
            textColorRGB=self.name_color,
            textSize=self.name_size,
            replaceItemUniqueId=self.name_id)

    def init_pose_data_draw(self):
        # 绘制位姿坐标
        self.text_xyz_data = self.pybullet.addUserDebugText(
            text="X:{:.2f}, Y:{:.2f}, Z:{:.2f}".format(0, 0, 0),
            textPosition=[-10, 1, 1],
            textColorRGB=[0, 0, 0],
            textSize=1.2,
        )
        # 绘制位姿坐标
        self.text_rpy_data = self.pybullet.addUserDebugText(
            text="roll:{:.2f}, pitch:{:.2f}, yaw:{:.2f}".format(0, 0, 0),
            textPosition=[-10, 1, 0],
            textColorRGB=[0, 0, 0],
            textSize=1.2,
        )

    def pose_data_draw(self, pose):
        # 绘制位姿坐标
        self.pybullet.addUserDebugText(
            text="X:{:.2f}, Y:{:.2f}, Z:{:.2f}".format(*pose.get_position()),
            textPosition=[-10, 1, 1],
            textColorRGB=[0, 0, 0],
            textSize=1.2,
            replaceItemUniqueId=self.text_xyz_data
        )
        # 绘制位姿坐标
        self.pybullet.addUserDebugText(
            text="roll:{:.2f}, pitch:{:.2f}, yaw:{:.2f}".format(*pose.get_euler_angle()),
            textPosition=[-10, 1, 0],
            textColorRGB=[0, 0, 0],
            textSize=1.2,
            replaceItemUniqueId=self.text_rpy_data
        )

    def init_angle_data_draw(self):
        # 绘制位姿坐标
        self.text_joint_u_data = self.pybullet.addUserDebugText(
            text="joint1:{:.2f}, joint2:{:.2f}, joint3:{:.2f}".format(0, 0, 0),
            textPosition=[-10, 1, 3],
            textColorRGB=[0, 0, 0],
            textSize=1.2,
        )
        # 绘制位姿坐标
        self.text_joint_b_data = self.pybullet.addUserDebugText(
            text="joint4:{:.2f}, joint5:{:.2f}, joint6:{:.2f}".format(0, 0, 0),
            textPosition=[-10, 1, 2],
            textColorRGB=[0, 0, 0],
            textSize=1.2,
        )

    def angle_data_draw(self, angle):
        # 绘制位姿坐标
        self.pybullet.addUserDebugText(
            text="joint1:{:.2f}, joint2:{:.2f}, joint3:{:.2f}".format(*angle[:3]),
            textPosition=[-10, 1, 3],
            textColorRGB=[0, 0, 0],
            textSize=1.2,
            replaceItemUniqueId=self.text_joint_u_data
        )
        # 绘制位姿坐标
        self.pybullet.addUserDebugText(
            text="joint4:{:.2f}, joint5:{:.2f}, joint6:{:.2f}".format(*angle[3:]),
            textPosition=[-10, 1, 2],
            textColorRGB=[0, 0, 0],
            textSize=1.2,
            replaceItemUniqueId=self.text_joint_b_data
        )

    def draw_path(self, posi_a, posi_b):
        """绘制轨迹-目标值"""
        # 绘制轨迹, 注意mm转m
        self.pybullet.addUserDebugLine(
            lineFromXYZ=[v * 0.001 for v in posi_a],
            lineToXYZ=[v * 0.001 for v in posi_b],
            lineColorRGB=[1.0, 1.0, 0],
            lineWidth=3)
