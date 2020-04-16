execute_process(COMMAND "/home/jmtc7/Learning/Autonomous-ROS-Car/ros/build/geometry2/tf2_py/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/jmtc7/Learning/Autonomous-ROS-Car/ros/build/geometry2/tf2_py/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
