execute_process(COMMAND "/home/martin/HRI/catkin_ws_v2/build/hlpr_speech/hlpr_speech_recognition/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/martin/HRI/catkin_ws_v2/build/hlpr_speech/hlpr_speech_recognition/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
