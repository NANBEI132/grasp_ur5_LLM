execute_process(COMMAND "/home/zzq/UR5-Pick-and-Place-Simulation/catkin_ws/build/ur_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/zzq/UR5-Pick-and-Place-Simulation/catkin_ws/build/ur_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
