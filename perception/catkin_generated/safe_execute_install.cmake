execute_process(COMMAND "/home/team4/catkin_ws/src/cse481wi19/perception/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/team4/catkin_ws/src/cse481wi19/perception/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
