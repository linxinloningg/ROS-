import numpy as np
from numpy.lib.arraysetops import unique
from transform import Transform, Posture
from math import pi, atan2, pow, sqrt, cos, sin

# 设置Numpy打印精度
np.set_printoptions(precision=3, suppress=True)


class DobotKinematic:
    """
    DH表
    0       0       theta1      0
    -pi/2   0       theta2      0
    0       a2      theta3      0
    0       a3      0           0
    a2, a3 在该示例中分别取80, 60
    """

    def __init__(self):
        pass

    @staticmethod
    def forward_kinematic(joint_angle):
        # 提取关节角度
        theta1, theta2, theta3 = joint_angle

        # 关节与关节之间的变换
        """
        DH表
        0       0       theta1      0
        -pi/2   0       theta2      0
        0       a2      theta3      0
        0       a3      0           0
        a2, a3 在该示例中分别取80, 60
        """
        right_angle = pi / 2
        t_01 = Transform.dhmat(0, 0, theta1, 0)
        t_12 = Transform.dhmat(-right_angle, 0, theta2, 0)
        t_23 = Transform.dhmat(0, 80, theta3, 0)
        t_34 = Transform.dhmat(0, 60, 0, 0)

        # 计算机械臂基坐标系到腕关节坐标系的变换
        t_02 = t_01.dot(t_12)
        t_03 = t_02.dot(t_23)
        t_04 = t_03.dot(t_34)

        # 创建位姿对象
        pose = Posture()
        # 从变换矩阵中提取x, y, z, roll, pitch, yaw
        pose.set_transform_matrix(t_04)
        return pose


class Arm4DoFKinematic:
    """
    DH表
    0       0       theta1      0
    -pi/2   0       theta2      0
    0       80      theta3      0
    0       76      theta4      0
    0       136     -pi/2       0
    -pi/2   0       0           0
    """

    def __init__(self):
        pass

    @staticmethod
    def forward_kinematic(joint_angle):
        """正向运动学-工具坐标系"""
        # 提取关节角度
        theta1, theta2, theta3, theta4 = joint_angle
        # 关节与关节之间的变换
        t_01 = Transform.dhmat(0, 0, theta1, 0)
        t_12 = Transform.dhmat(-pi / 2, 0, theta2, 0)
        t_23 = Transform.dhmat(0, 80, theta3, 0)
        t_34 = Transform.dhmat(0, 76, theta4, 0)
        t_45 = Transform.dhmat(0, 136, -pi / 2, 0)
        t_56 = Transform.dhmat(pi / 2, 0, 0, 0)
        # 计算机械臂基坐标系到腕关节坐标系的变换
        t_02 = t_01.dot(t_12)
        t_03 = t_02.dot(t_23)
        t_04 = t_03.dot(t_34)
        t_05 = t_04.dot(t_45)
        t_06 = t_05.dot(t_56)
        # 创建位姿对象
        pose = Posture()
        pose.set_transform_matrix(t_06)
        return pose


class UR5Kinematic:
    """
    DH表
    0       0       theta1      163
    -pi/2   0       theta2      0
    0       425     theta3      0
    0       392     theta4      134
    pi/2    0       theta5      -100
    -pi/2   0       theta6      100
    """

    def __init__(self):
        # 默认关节角度(0, 1, 2, 3, 4, 5)
        self.JOINT_ANGLE_DEFAULT = [0 for _ in range(6)]
        # 关节角度下限, 单位: 弧度
        self.JOINT_ANGLE_LOWERB = [-np.pi for _ in range(6)]
        # 关节角度上限制, 单位: 弧度
        self.JOINT_ANGLE_UPPERB = [np.pi for _ in range(6)]

        # 连杆长度
        self.D1 = 163
        self.A2 = 425
        self.A3 = 392
        self.D4 = 134
        self.D5 = -100
        self.D6 = 100

    @staticmethod
    def forward_kinematic(joint_angle):
        """

        :param joint_angle:
        :return:
        """
        # 提取关节角度
        theta1, theta2, theta3, theta4, theta5, theta6 = joint_angle

        # 关节与关节之间的变换
        """
            DH表
            0       0       theta1      163
            -pi/2   0       theta2      0
            0       425     theta3      0
            0       392     theta4      134
            pi/2    0       theta5      -100
            -pi/2   0       theta6      100
        """
        right_angle = pi / 2
        t_01 = Transform.dhmat(0, 0, theta1, 163)
        t_12 = Transform.dhmat(-right_angle, 0, theta2, 0)
        t_23 = Transform.dhmat(0, 425, theta3, 0)
        t_34 = Transform.dhmat(0, 392, theta4, 134)
        t_45 = Transform.dhmat(right_angle, 0, theta5, -100)
        t_56 = Transform.dhmat(-right_angle, 0, theta6, 100)

        # 计算机械臂基坐标系到腕关节坐标系的变换
        t_02 = t_01.dot(t_12)
        t_03 = t_02.dot(t_23)
        t_04 = t_03.dot(t_34)
        t_05 = t_04.dot(t_45)
        t_06 = t_05.dot(t_56)

        # 创建位姿对象
        pose = Posture()
        # 从变换矩阵中提取x, y, z, roll, pitch, yaw
        pose.set_transform_matrix(t_06)
        return pose

    def inverse_kinematic(self, pose, last_joint_angle=None):
        """逆向运动学 8组解"""
        # 上一次的关节角度
        if last_joint_angle is None:
            last_joint_angle = self.JOINT_ANGLE_DEFAULT
        # 候选关节角度
        candi_joint_angle_list = []
        # 提取腕关节的坐标
        x6, y6, z6 = pose.get_position()
        # 旋转矩阵
        rmat = pose.get_rotation_matrix()
        r11, r12, r13, r21, r22, r23, r31, r32, r33 = rmat.reshape(-1)
        #  求解P05:  关节5坐标系原点在基坐标系下的坐标
        p06 = np.float32([x6, y6, z6])
        z6_unit = np.float32([r13, r23, r33])  # 关节6坐标系Z轴单位向量
        p05 = p06 - z6_unit * self.D6  # 关节5坐标系原点在基坐标系下的坐标
        x5, y5, z5 = p05
        # 求解theta1: 关节1的角度
        alpha1 = atan2(y5, x5)
        c_pow2 = pow(x5, 2) + pow(y5, 2)
        # c = sqrt(c_pow2)
        b_pow2 = c_pow2 - pow(self.D4, 2)
        if b_pow2 <= 0:
            # 不再工作区内，无解
            return []
        b = sqrt(b_pow2)
        alpha2_abs = atan2(self.D4, b)
        # 右手模式
        theta1_1 = alpha1 - alpha2_abs
        # 左手模式
        theta1_2 = alpha1 + alpha2_abs + pi
        if theta1_2 > pi:
            theta1_2 -= 2 * pi
        # 求解theta5: 关节5的角度
        for theta1 in unique([theta1_1, theta1_2]):
            c1 = cos(theta1)
            s1 = sin(theta1)
            t10 = np.float32([
                [c1, s1, 0, 0],
                [-s1, c1, 0, 0],
                [0, 0, 1, -self.D1],
                [0, 0, 0, 1]])
            t06 = pose.get_transform_matrix()
            t16 = np.dot(t10, t06)
            # 提取旋转矩阵
            # 计算^{1}_{6}R: 关节6坐标系在关节1坐标系下的旋转矩阵
            r16 = t16[:3, :3]
            q11, q12, q13, q21, q22, q23, q31, q32, q33 = r16.reshape(-1)
            # 求解theta5
            c5 = q23
            s5_abs = sqrt(1 - c5 ** 2)
            theta5_1 = atan2(s5_abs, c5)
            theta5_2 = atan2(-s5_abs, c5)
            # 求解 theta6 与 theta234
            for theta5 in unique([theta5_1, theta5_2]):
                s5 = sin(theta5)
                if s5 == 0:
                    if theta5 == 0:
                        # 万向锁
                        theta6 = last_joint_angle[5]
                        theta234 = atan2(-q12, q11) - theta6
                    else:
                        # 万向锁 theta5 = pi 或 -pi
                        # 注: 实际上 -\pi跟\pi指代的是同一个角度
                        theta6 = last_joint_angle[5]
                        theta234 = atan2(q31, -q32) + theta6
                else:
                    # 一般情况
                    if s5 > 0:
                        theta234 = atan2(q33, -q13)
                        theta6 = atan2(-q22, q21)
                    else:
                        theta234 = atan2(-q33, q13)
                        theta6 = atan2(q22, -q21)

                # 求解theta3
                s234 = sin(theta234)
                c234 = cos(theta234)

                # 计算关节5坐标系原点在关节1坐标系下的位置
                s6 = sin(theta6)
                c6 = cos(theta6)
                t65 = np.float32([
                    [c6, 0, -s6, 0],
                    [-s6, 0, -c6, 0],
                    [0, 1, 0, -self.D6],
                    [0, 0, 0, 1]])
                t15 = np.dot(t16, t65)
                x5_1, y5_1, z5_1 = t15[:3, 3]

                b1 = x5_1 - self.D5 * s234
                b2 = -z5_1 + self.D5 * c234
                c3 = (b1 ** 2 + b2 ** 2 - self.A2 ** 2 - self.A3 ** 2) / (2 * self.A2 * self.A3)
                if abs(c3) > 1:
                    # 非法cos值, 工作区达不到
                    continue
                s3_abs = sqrt(1 - c3 ** 2)
                theta3_1 = atan2(s3_abs, c3)
                theta3_2 = atan2(-s3_abs, c3)

                # 求解theta2
                for theta3 in unique([theta3_1, theta3_2]):
                    k1 = self.A2 + self.A3 * c3
                    k2 = self.A3 * sin(theta3)
                    theta2 = atan2(b2, b1) - atan2(k2, k1)
                    # 约束theta2的范围
                    # 将theta2缩放到 -pi, pi之间
                    if theta2 > pi:
                        theta2 -= 2 * pi
                    elif theta2 < -pi:
                        theta2 += 2 * pi

                    # 求解theta4
                    theta4 = theta234 - theta2 - theta3
                    # 约束theta4的范围
                    if theta4 > pi:
                        theta4 -= 2 * pi
                    elif theta4 < -pi:
                        theta4 += 2 * pi

                    candi_joint_angle_list.append([theta1, theta2, theta3, theta4, theta5, theta6])

        return candi_joint_angle_list

    """逆向运动学，添加约束减少逆解个数"""

    def inverse_kinematic_constraint(self, pose, last_joint_angle=None):
        """逆向运动学，添加约束减少逆解个数
        - theta2 < 0
        - theta3 > 0
        """
        # 上一次的关节角度
        if last_joint_angle is None:
            last_joint_angle = self.JOINT_ANGLE_DEFAULT
        # 候选关节角度
        candi_joint_angle_list = []
        # 提取腕关节的坐标
        x6, y6, z6 = pose.get_position()
        # 旋转矩阵
        rmat = pose.get_rotation_matrix()
        r11, r12, r13, r21, r22, r23, r31, r32, r33 = rmat.reshape(-1)
        #  求解P05:  关节5坐标系原点在基坐标系下的坐标
        p06 = np.float32([x6, y6, z6])
        z6_unit = np.float32([r13, r23, r33])  # 关节6坐标系Z轴单位向量
        p05 = p06 - z6_unit * self.D6  # 关节5坐标系原点在基坐标系下的坐标
        x5, y5, z5 = p05
        # 求解theta1: 关节1的角度
        alpha1 = atan2(y5, x5)
        c_pow2 = pow(x5, 2) + pow(y5, 2)
        # c = sqrt(c_pow2)
        b_pow2 = c_pow2 - pow(self.D4, 2)
        if b_pow2 <= 0:
            # 不再工作区内，无解
            return []
        b = sqrt(b_pow2)

        alpha2_abs = atan2(self.D4, b)
        # 右手模式
        theta1_1 = alpha1 - alpha2_abs
        # 左手模式
        theta1_2 = alpha1 + alpha2_abs + pi
        if theta1_2 > pi:
            theta1_2 -= 2 * pi
        # 求解theta5: 关节5的角度
        for theta1 in unique([theta1_1, theta1_2]):
            c1 = cos(theta1)
            s1 = sin(theta1)
            t10 = np.float32([
                [c1, s1, 0, 0],
                [-s1, c1, 0, 0],
                [0, 0, 1, -self.D1],
                [0, 0, 0, 1]])
            t06 = pose.get_transform_matrix()
            t16 = np.dot(t10, t06)
            # 提取旋转矩阵
            # 计算^{1}_{6}R: 关节6坐标系在关节1坐标系下的旋转矩阵
            r16 = t16[:3, :3]
            q11, q12, q13, q21, q22, q23, q31, q32, q33 = r16.reshape(-1)
            # 求解theta5
            c5 = q23
            s5_abs = sqrt(1 - c5 ** 2)
            theta5_1 = atan2(s5_abs, c5)
            theta5_2 = atan2(-s5_abs, c5)
            # 求解 theta6 与 theta234
            for theta5 in unique([theta5_1, theta5_2]):
                s5 = sin(theta5)
                if s5 == 0:
                    if theta5 == 0:
                        # 万向锁
                        theta6 = last_joint_angle[5]
                        theta234 = atan2(-q12, q11) - theta6
                    else:
                        # 万向锁 theta5 = pi 或 -pi
                        # 注: 实际上 -\pi跟\pi指代的是同一个角度
                        theta6 = last_joint_angle[5]
                        theta234 = atan2(q31, -q32) + theta6
                else:
                    # 一般情况
                    if s5 > 0:
                        theta234 = atan2(q33, -q13)
                        theta6 = atan2(-q22, q21)
                    else:
                        theta234 = atan2(-q33, q13)
                        theta6 = atan2(q22, -q21)

                # 求解theta3
                s234 = sin(theta234)
                c234 = cos(theta234)

                # 计算关节5坐标系原点在关节1坐标系下的位置
                s6 = sin(theta6)
                c6 = cos(theta6)
                t65 = np.float32([
                    [c6, 0, -s6, 0],
                    [-s6, 0, -c6, 0],
                    [0, 1, 0, -self.D6],
                    [0, 0, 0, 1]])
                t15 = np.dot(t16, t65)
                x5_1, y5_1, z5_1 = t15[:3, 3]

                b1 = x5_1 - self.D5 * s234
                b2 = -z5_1 + self.D5 * c234
                c3 = (b1 ** 2 + b2 ** 2 - self.A2 ** 2 - self.A3 ** 2) / (2 * self.A2 * self.A3)
                if abs(c3) > 1:
                    # 非法cos值, 工作区达不到
                    continue
                s3 = sqrt(1 - c3 ** 2)
                # 只要正手解
                theta3 = atan2(s3, c3)

                # 求解theta2
                if theta3 < 0:
                    continue
                k1 = self.A2 + self.A3 * c3
                k2 = self.A3 * sin(theta3)
                theta2 = atan2(b2, b1) - atan2(k2, k1)
                # 约束theta2的范围
                # 将theta2缩放到 -pi, pi之间
                if theta2 > pi:
                    theta2 -= 2 * pi
                elif theta2 < -pi:
                    theta2 += 2 * pi

                if theta2 > 0:
                    continue
                # 约束theta4的范围
                theta4 = theta234 - theta2 - theta3
                if theta4 > pi:
                    theta4 -= 2 * pi
                elif theta4 < -pi:
                    theta4 += 2 * pi

                candi_joint_angle_list.append([theta1, theta2, theta3, theta4, theta5, theta6])

        return candi_joint_angle_list
