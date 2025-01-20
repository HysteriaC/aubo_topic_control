cmake_minimum_required(VERSION 3.0.2)
project(aubo_robot_driver)

## 编译依赖
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

## 声明catkin包
catkin_package()

## 指定脚本安装位置
catkin_install_python(PROGRAMS
  scripts/aubo_driver_ros.py
  scripts/joint_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 将动态库直接复制到 devel/lib/${PROJECT_NAME} 目录
file(GLOB LIB_FILES "${CMAKE_CURRENT_SOURCE_DIR}/lib/aubosdk/python64/*")
foreach(LIB_FILE ${LIB_FILES})
    file(COPY ${LIB_FILE}
         DESTINATION ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})
endforeach()

# 检查 /opt/config 是否存在
if(NOT EXISTS /opt/config)
    # 创建目录并设置权限
    add_custom_target(create_config_directory ALL
        COMMAND sudo mkdir -p /opt/config/urdf
        COMMAND sudo chmod -R 775 /opt/config/urdf
        COMMENT "Creating /opt/config/urdf and setting permissions"
    )
    if(NOT EXISTS /opt/config/urdf)
        # 将 urdf 文件夹复制到 /opt/config/urdf
        add_custom_target(copy_urdf ALL
            COMMAND sudo ${CMAKE_COMMAND} -E copy_directory
        	${CMAKE_CURRENT_SOURCE_DIR}/aubo_description/urdf
        	/opt/config/urdf
            COMMENT "Copying urdf files from aubo_description to /opt/config/urdf"
        )
    else()
        message(STATUS "/opt/config/urdf already exists. Skipping copy.")
    endif()
else()
    message(STATUS "/opt/config already exists. Skipping creation.")
    if(NOT EXISTS /opt/config/urdf)
        # 将 urdf 文件夹复制到 /opt/config/urdf
        add_custom_target(copy_urdf ALL
            COMMAND sudo ${CMAKE_COMMAND} -E copy_directory
        	${CMAKE_CURRENT_SOURCE_DIR}/aubo_description/urdf
        	/opt/config/urdf
            COMMENT "Copying urdf files from aubo_description to /opt/config/urdf"
        )
    else()
        message(STATUS "/opt/config/urdf already exists. Skipping copy.")
    endif()
endif()

