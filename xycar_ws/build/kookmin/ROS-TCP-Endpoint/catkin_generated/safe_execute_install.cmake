execute_process(COMMAND "/home/taeyeong/xytron_404/xycar_ws/build/kookmin/ROS-TCP-Endpoint/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/taeyeong/xytron_404/xycar_ws/build/kookmin/ROS-TCP-Endpoint/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
