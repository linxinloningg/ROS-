# 基础

>* 3D空间下刚体的位姿描述
>* 机械臂坐标系系统

# 机械臂运动学

>- **正向运动学** ：给定每个关节的旋转角度，求解末端执行器的位姿。
>- **逆向运动学** ：给定末端执行器的位姿，求解每个关节的旋转角度。
>
>### 正向运动学
>
>>如何编写一个机械臂正向运动学解析器？
>>
>>**我们将机械臂的每一个关节轴，都建立一个坐标系，那么从关节1到关节0的变换，其实就是做了一次刚体的坐标变换。 而关节7的末端点，则是串着做了好多次的坐标变换。**
>>
>>对机械臂求逆解的方法分为两类：封闭解法与数值解法。
>>
>>由于数值解法是通过迭代求解的，因此速度较慢；
>>
>>封闭解法是以解析的方式求取逆解，分为几何法与代数法，得到的解称为解析解。
>>
>>* 了解3D空间下向量的平移和旋转
>> * 平移矩阵
>> * 旋转矩阵
>> * 变换矩阵
>>* 了解机械臂节点的坐标变换
>>* 了解旋转矩阵转变为欧拉角
>>
>>代码示例：Kinematic_Test.ipynb
>>
>>```python
>>from kinematic import UR5Kinematic, DobotKinematic, Arm4DoFKinematic
>>># UR5机械臂正向运动学解析
>>joint_angle = [0 for _ in range(6)]
>>pose = UR5Kinematic.forward_kinematic(joint_angle)
>>print(pose)
>>"""
>>Pose x=817.000, y=234.000, z=63.000, roll=-90.000, pitch=-0.000, yaw=0.000
>>"""
>>
>>># Dobot机械臂正向运动学解析
>>joint_angle = [0 for _ in range(3)]
>>pose = DobotKinematic.forward_kinematic(joint_angle)
>>print(pose)
>>"""
>>Pose x=140.000, y=0.000, z=0.000, roll=-90.000, pitch=-0.000, yaw=0.000
>>"""
>>
>>># Arm4DoF机械臂正向运动学解析
>>joint_angle = [0 for _ in range(4)]
>>pose = Arm4DoFKinematic.forward_kinematic(joint_angle)
>>print(pose)
>>"""
>>Pose x=292.000, y=0.000, z=0.000, roll=0.000, pitch=-90.000, yaw=-0.000
>>"""
>>```
>
>### 逆向运动学
>
>>如何编写一个机械臂逆向运动学解析器？
>
>>>位姿（X, Y, Z, roll, pitch, yaw）
>
>>>* 逆向运动学求解的方法主要分为如下三种：
>>
>>  1. 几何法
>>  2. 代数法
>>  3. 数值法 (迭代)
>>
>>* 了解欧拉角转变为旋转矩阵
>>>* 对机械臂进行逆向运动学分析（以ur5机械臂举例）
>
>>>代码示例：Kinematic_Test.ipynb
>
>>```python
>>from kinematic import UR5Kinematic
>>target_pose = Posture()
>>target_pose.set_position(817, 234, 63)
>>target_pose.set_euler_angle(radians(-90.000), 0, 0)
>>print(pose)
>>"""
>>Pose x=817.000, y=234.000, z=63.000, roll=-90.000, pitch=-0.000, yaw=0.000]
>>"""
>>candi_joint_angle_list = UR5Kinematic.inverse_kinematic(target_pose)
>>print(f"candi_joint_angle_list len {len(candi_joint_angle_list)}")
>>for joint_angle in candi_joint_angle_list:
>>pose = UR5Kinematic.forward_kinematic(joint_angle)
>>print("Joint Angle: " + ",".join(["{:.1f}".format(degrees(angle)) for angle in  joint_angle]))
>>    >print(pose)
>>"""
>>candi_joint_angle_list len 3
>>Joint Angle: -161.4,-180.0,-0.0,-180.0,161.4,0.0
>>Pose x=817.000, y=234.000, z=63.000, roll=-90.000, pitch=0.000, yaw=-0.000]
>>Joint Angle: -161.4,180.0,0.0,180.0,161.4,0.0
>>Pose x=817.000, y=234.000, z=63.000, roll=-90.000, pitch=0.000, yaw=-0.000]
>>Joint Angle: 0.0,0.0,0.0,-0.0,0.0,0.0
>>Pose x=817.000, y=234.000, z=63.000, roll=-90.000, pitch=-0.000, yaw=0.000]
>>"""
>>>candi_joint_angle_list = UR5Kinematic.inverse_kinematic_constraint(target_pose)
>>print(f"candi_joint_angle_list len {len(candi_joint_angle_list)}")
>>for joint_angle in candi_joint_angle_list:
>>pose = UR5Kinematic.forward_kinematic(joint_angle)
>>print("Joint Angle: " + ",".join(["{:.1f}".format(degrees(angle)) for angle in  joint_angle]))
>>print(pose)
>>"""
>>candi_joint_angle_list len 1
>>Joint Angle: 0.0,0.0,0.0,-0.0,0.0,0.0
>>Pose x=817.000, y=234.000, z=63.000, roll=-90.000, pitch=-0.000, yaw=0.000]
>>"""
>>```
>
>>