# check the existance of OpenCV, if not/outdated, install/reinstall
find_package(OpenCV REQUIRED)
if (${OpenCV_FOUND})
	message(STATUS "OpenCV found")
else()
	message(STATUS "OpenCV not found! Executing install script.")
	execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh
	    		RESULT_VARIABLE retcode)
	if(NOT ${retcode} EQUAL 0)
	    message(FATAL_ERROR "opencv.cmake: Error when excuting ${CMAKE_CURRENT_LIST_DIR}/install.sh")
	endif()
	find_package(OpenCV REQUIRED) # try and find again
endif()
include_directories(${OpenCV_INCLUDE_DIR})

#set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}  /PATH/TO/opencv/share/OpenCV/)
#set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

mark_as_advanced(OpenCV_DIR)
