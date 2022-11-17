# 开始使用

* ## 1.创建工作空间并初始化

  >```bash
  >mkdir -p 自定义空间名称/src
  >cd 自定义空间名称
  >catkin_make
  >```
  >
  >如果系统装有anaconda虚拟环境，有可能catkin_make会索引到虚拟环境中python，这时候需要特别指定：
  >
  >```bash
  >catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
  >```

* ## 2.进入 src 创建 ros 包并添加依赖

  >```bash
  >cd src
  ># catkin_create_pkg [功能包名称] [依赖功能包1] [依赖功能包n]
  >catkin_create_pkg 自定义ROS包名 roscpp rospy std_msgs
  >```
  >
  >其中roscpp是使用C++实现的库，而rospy则是使用python实现的库，std_msgs是标准消息库，创建ROS功能包时，一般都会依赖这三个库实现。
  >
  >```bash
  >$ cd 自定义ROS包名
  >$ ls
  >include → include目录
  >src → 源代码目录
  >CMakeLists.txt → 构建配置文件
  >package.xml → 功能包配置文件 
  >```
  >
  >*其余如果在构建中需要另外的库可在cmakelists.txt中添加，也可在创建包时添加。*

* ## 3.编写python脚本代码

  >1. 进入 ros 包添加 scripts 目录并编辑 python 文件
  >
  >```bash
  >cd ros包
  >mkdir scripts
  >```
  >
  >```bash
  >新建 python 文件: (文件名自定义)
  >```
  >
  >2. 为 python 文件添加可执行权限
  >
  >   ```bash
  >   chmod +x 自定义文件名.py
  >   ```
  >
  >3. 编辑 ros 包下的 CamkeList.txt 文件
  >
  >   ```bash
  >   catkin_install_python(PROGRAMS 
  >     scripts/自定义文件名.py
  >     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  >   )
  >   ```

* ## 4.进入工作空间目录并编译

  >```bash
  >cd 自定义空间名称
  >catkin_make
  >```

* ## 5.使用

  >```bash
  >先启动命令行1：
  >
  >roscore
  >
  >再启动命令行2：
  >cd 工作空间
  >source ./devel/setup.bash
  >rosrun 包名 自定义文件名.py
  >```

  ## HelloWorld

  >1. 先创建一个工作空间；
  >2. 再创建一个功能包；
  >3. 编辑源文件；
  >4. 编辑配置文件；
  >5. 编译并执行。
  >
  >#### 1.进入 ros 包添加 scripts 目录并编辑 python 文件
  >
  >```
  >cd ros包
  >mkdir scripts
  >```
  >
  >新建 python 文件: (文件名自定义)
  >
  >```py
  >#! /usr/bin/env python
  >
  >"""
  >    Python 版 HelloWorld
  >
  >"""
  >import rospy
  >
  >if __name__ == "__main__":
  >    rospy.init_node("Hello")
  >    rospy.loginfo("Hello World!!!!")
  >```
  >
  >#### 2.为 python 文件添加可执行权限
  >
  >```
  >chmod +x 自定义文件名.py
  >```
  >
  >#### 3.编辑 ros 包下的 CamkeList.txt 文件
  >
  >```
  >catkin_install_python(PROGRAMS scripts/自定义文件名.py
  >  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  >)
  >```
  >
  >#### 4.进入工作空间目录并编译
  >
  >```
  >cd 自定义空间名称
  >catkin_make
  >```
  >
  >#### 5.进入工作空间目录并执行
  >
  >**先启动命令行1：**
  >
  >```
  >roscore
  >```
  >
  >**再启动命令行2：**
  >
  >```
  >cd 工作空间
  >source ./devel/setup.bash
  >rosrun 包名 自定义文件名.py
  >```
  >
  >输出结果:`Hello World!!!!`

  