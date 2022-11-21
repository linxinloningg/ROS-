from numpy import set_printoptions, float32, eye, array, sqrt, copy, degrees
from math import sin, cos, atan2, pi

# 设置Numpy打印精度
set_printoptions(precision=3, suppress=True)


class Transform:
    def __init__(self):
        pass

    @staticmethod
    def translation_x(dx):
        """沿X轴平移"""
        return float32([
            [1, 0, 0, dx],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

    @staticmethod
    def translation_y(dy):
        """沿Y轴平移"""
        return float32([
            [1, 0, 0, 0],
            [0, 1, 0, dy],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

    @staticmethod
    def translation_z(dz):
        """沿Z轴平移"""
        return float32([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, dz],
            [0, 0, 0, 1]])

    @staticmethod
    def rotate_x(gamma):
        """绕X轴旋转"""
        return array([
            [1, 0, 0, 0],
            [0, cos(gamma), -sin(gamma), 0],
            [0, sin(gamma), cos(gamma), 0],
            [0, 0, 0, 1]])

    @staticmethod
    def rotate_y(beta):
        """绕Y轴旋转"""
        return array([
            [cos(beta), 0, sin(beta), 0],
            [0, 1, 0, 0],
            [-sin(beta), 0, cos(beta), 0],
            [0, 0, 0, 1]])

    @staticmethod
    def rotate_z(alpha):
        """绕Z轴旋转"""
        return array([
            [cos(alpha), -sin(alpha), 0, 0],
            [sin(alpha), cos(alpha), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

    @staticmethod
    def dhmat(alpha, a, theta, d):
        """
        DH变换矩阵
        :param alpha:绕X轴旋转角度
        :param a:绕X轴平移距离
        :param theta:绕Z轴旋转角度
        :param d:绕Z轴平移距离
        :return:
        """
        result = Transform.rotate_x(alpha)
        result = result.dot(Transform.translation_x(a))
        result = result.dot(Transform.rotate_z(theta))
        result = result.dot(Transform.translation_z(d))
        return result

    @staticmethod
    def euler2rmat(roll, pitch, yaw):
        """欧拉角转换为旋转矩阵"""
        alpha, beta, gamma = yaw, pitch, roll
        cos_gamma = cos(gamma)
        sin_gamma = sin(gamma)
        cos_beta = cos(beta)
        sin_beta = sin(beta)
        cos_alpha = cos(alpha)
        sin_alpha = sin(alpha)

        r11 = cos_alpha * cos_beta
        r12 = -sin_alpha * cos_gamma + sin_beta * sin_gamma * cos_alpha
        r13 = sin_alpha * sin_gamma + sin_beta * cos_alpha * cos_gamma
        r21 = sin_alpha * cos_beta
        r22 = sin_alpha * sin_beta * sin_gamma + cos_alpha * cos_gamma
        r23 = sin_alpha * sin_beta * cos_gamma - sin_gamma * cos_alpha
        r31 = -sin_beta
        r32 = sin_gamma * cos_beta
        r33 = cos_beta * cos_gamma
        return array([
            [r11, r12, r13],
            [r21, r22, r23],
            [r31, r32, r33]])

    # 旋转矩阵转换为欧拉角
    @staticmethod
    def rmat2euler(rmat):
        """旋转矩阵转换为欧拉角"""

        r11, r12, r13, r21, r22, r23, r31, r32, r33 = rmat.reshape(-1)
        if abs(r31) >= (1 - 0.000001):
            # 出现万向锁的问题
            if r31 < 0:
                gamma = 0
                beta = pi / 2
                alpha = atan2(r23, r22)
                return [[gamma, beta, alpha]]
            else:
                gamma = 0
                beta = -pi / 2
                alpha = atan2(-r23, r22)
                return [[gamma, beta, alpha]]
        else:
            # 正常求解
            cos_beta = sqrt(r32 * r32 + r33 * r33)
            cos_beta_list = [cos_beta, -cos_beta]
            rpy_list = []
            for cos_beta in cos_beta_list:
                beta = atan2(-r31, cos_beta)
                alpha = atan2(r21 / cos_beta, r11 / cos_beta)
                gamma = atan2(r32 / cos_beta, r33 / cos_beta)
                rpy_list.append([gamma, beta, alpha])

            return rpy_list


class Posture:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.rmat = eye(3)

    # 设置位置
    def set_position(self, x, y, z):
        """设置位置"""
        self.x = x
        self.y = y
        self.z = z

    # 获取位置
    def get_position(self):
        """获取位置"""
        return [self.x, self.y, self.z]

    # 设置欧拉角
    def set_euler_angle(self, roll, pitch, yaw):
        """设置欧拉角"""
        # 赋值欧拉角
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        # 更新旋转矩阵，欧拉角变为旋转矩阵
        self.rmat = Transform.euler2rmat(roll=self.roll, pitch=self.pitch, yaw=self.yaw)

    # 获取欧拉角
    def get_euler_angle(self):
        """获取欧拉角"""
        return [self.roll, self.pitch, self.yaw]

    # 设置旋转矩阵
    def set_rotation_matrix(self, rmat):
        """设置旋转矩阵"""
        self.rmat = copy(rmat)
        # 同步更新欧拉角，旋转矩阵变为欧拉角
        self.roll, self.pitch, self.yaw = Transform.rmat2euler(rmat)[0]

    # 获取旋转矩阵
    def get_rotation_matrix(self):
        """获取旋转矩阵"""
        return Transform.euler2rmat(
            roll=self.roll, pitch=self.pitch, yaw=self.yaw)

    # 设置变换矩阵
    def set_transform_matrix(self, tmat):
        """设置旋转矩阵"""
        x, y, z = tmat[:3, 3].reshape(-1)
        self.set_position(x, y, z)
        rmat = tmat[:3, :3]
        self.set_rotation_matrix(rmat)

    # 获取变换矩阵
    def get_transform_matrix(self):
        """获取变换矩阵"""
        tmat = eye(4)
        tmat[0, 3] = self.x
        tmat[1, 3] = self.y
        tmat[2, 3] = self.z
        tmat[:3, :3] = self.rmat
        return tmat

    def print_posture(self):
        params = [self.x, self.y, self.z, degrees(self.roll),
                  degrees(self.pitch), degrees(self.yaw)]
        print("Pose x={:.3f}, y={:.3f}, z={:.3f}, roll={:.3f}, pitch={:.3f}, yaw={:.3f}]".format(*params))

    def __str__(self):
        # np.degree ()的使用 将弧度转化为角度
        params = [self.x, self.y, self.z, degrees(self.roll),
                  degrees(self.pitch), degrees(self.yaw)]
        return "Pose x={:.3f}, y={:.3f}, z={:.3f}, roll={:.3f}, pitch={:.3f}, yaw={:.3f}]".format(*params)
