解析：

* cmake_minimum_required(VERSION 2.8.3) 

  * 操作系统中安 装的cmake的最低版本。由于它目前被指定为版本2.8.3，所以如果使用低于此版本的 cmake，则必须更新版本。

* project(my_first_ros_pkg)

  * project项是功能包的名称。只需使用用户在package.xml中输入的功能包名即可。 请注意，如果功能包名称与package.xml中的标记中描述的功能包名称不同，则 在构建时会发生错误，因此需要注意。

* ```bash
  find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
  )
  ```

  * find_package项是进行构建所需的组件包。目前，roscpp和std_msgs被添加为依赖 包。如果此处没有输入功能包名称，则在构建时会向用户报错。换句话说，这是让用户先 创建依赖包的选项。

* ```bash
  ## Mark executable scripts (Python etc.) for installation
  ## in contrast to setup.py, you can choose the destination
  catkin_install_python(PROGRAMS
      scripts/topic_posting.py
      scripts/topic_subscriptions.py
      scripts/service_invocation.py
      scripts/parameter_settings.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  ```

  * 添加python脚本文件的选项



* catkin_python_setup()

  * catkin_python_setup( )选项是在使用Python，也就是使用rospy时的配置选项。其 功能是调用Python安装过程setup.py。

* ```bash
  add_message_files(
  FILES
  Message1.msg
  Message2.msg
  )
  ```

  * add_message_files是添加消息文件的选项。FILES将引用当前功能包目录的msg目 录中的*.msg文件，自动生成一个头文件（*.h）。在这个例子中，我们将使用消息文件 Message1.msg和Message2.msg。

* ```bash
  add_service_files(
  FILES
  Service1.srv
  Service2.srv
  )
  ```

  * add_service_files是添加要使用的服务文件的选项。使用FILES会引用功能包目录 中的srv目录中的*.srv文件。在这个例子中，用户可以选择使用服务文件Service1.srv和 Service2.srv。

* ```bash
  generate_messages(
   DEPENDENCIES
   std_msgs
  )
  ```

  * generate_messages是设置依赖的消息的选项。此示例是将DEPENDENCIES选项设 置为使用std_msgs消息包

* ```bash
  generate_dynamic_reconfigure_options(
   cfg/DynReconf1.cfg
   cfg/DynReconf2.cfg
  )
  ```

  * generate_dynamic_reconfigure_options是使用dynamic_reconfigure时加载要 引用的配置文件的设置

* ```bash
  catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_first_ros_pkg
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
  )
  ```

  * catkin 构建选项。INCLUDE_DIRS表示将使用INCLUDE_DIRS后面的内部 目录include的头文件。LIBRARIES表示将使用随后而来的功能包的库。
  *  CATKIN_DEPENDS后面指定如roscpp或std_msgs等依赖包。目前的设置是表示依 赖于roscpp和std_msgs。DEPENDS是一个描述系统依赖包的设置

* ```bash
  include_directories(
   ${catkin_INCLUDE_DIRS}
  )
  ```

  * include_directories是可以指定包含目录的选项。目前设定为${catkin_INCLUDE_ DIRS}，这意味着将引用每个功能包中的include目录中的头文件。

* ```bash
  add_library(my_first_ros_pkg
   src/${PROJECT_NAME}/my_first_ros_pkg.cpp
  )
  ```

  * add_dependencies是在构建该库和可执行文件之前，如果有需要预先生成的有依赖 性的消息或dynamic_reconfigure，则要先执行。以下内容是优先生成my_first_ros_ pkg库依赖的消息及dynamic reconfigure的设置

* ```bash
  add_dependencies(my_first_ros_pkg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  ```

  * add_executable是对于构建之后要创建的可执行文件的选项。

* ```bash
  add_executable(my_first_ros_pkg_node src/my_first_ros_pkg_node.cpp)
  ```

* ```bash
  target_link_libraries(my_first_ros_pkg_node
   ${catkin_LIBRARIES}
  ) 
  ```

  * target_link_libraries是在创建特定的可执行文件之前将库和可执行文件进行链接的 选项。

```bash
cmake_minimum_required(VERSION 3.0.2) #所需 cmake 版本
project(demo01_hello_vscode) #包名称，会被 ${PROJECT_NAME} 的方式调用

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#设置构建所需要的软件包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
#默认添加系统依赖
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# 启动 python 模块支持
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
## 声明 ROS 消息、服务、动作... ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# 生成消息、服务时的依赖包
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
## 声明 ROS 动态参数配置 ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
## catkin 特定配置##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# 运行时依赖
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo01_hello_vscode
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# 添加头文件路径，当前程序包的头文件路径位于其他文件路径之前
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# 声明 C++ 库
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/demo01_hello_vscode.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# 添加库的 cmake 目标依赖
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# 声明 C++ 可执行文件
add_executable(Hello_VSCode src/Hello_VSCode.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#重命名c++可执行文件
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#添加可执行文件的 cmake 目标依赖
add_dependencies(Hello_VSCode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#指定库、可执行文件的链接库
target_link_libraries(Hello_VSCode
  ${catkin_LIBRARIES}
)

#############
## Install ##
## 安装 ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#设置用于安装的可执行脚本
catkin_install_python(PROGRAMS
  scripts/Hi.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_demo01_hello_vscode.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

```



