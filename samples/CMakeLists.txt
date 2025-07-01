#set(SAMPLE_LIBRARIES 
#    calibration)

# Set the C++ standard to use
set(CMAKE_CXX_STANDARD 17)
add_definitions(-D NOMINMAX)
 
find_package(DvsenseDriver COMPONENTS Driver REQUIRED)

#file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rgb_frames DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/calib_data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})


#add_executable(rgb_single_calib rgb_single_calib.cpp)
#target_link_libraries(rgb_single_calib ${SAMPLE_LIBRARIES})
#target_include_directories(rgb_single_calib PUBLIC ${CALIB_INCLUDE_DIRS})

#add_executable(rgb_stereo_calib rgb_stereo_calib.cpp)
#target_link_libraries(rgb_stereo_calib ${SAMPLE_LIBRARIES})
#target_include_directories(rgb_stereo_calib PUBLIC ${CALIB_INCLUDE_DIRS})

#add_executable(load_from_json load_from_json.cpp)
#target_link_libraries(load_from_json ${SAMPLE_LIBRARIES})
#target_include_directories(load_from_json PUBLIC ${CALIB_INCLUDE_DIRS})

add_subdirectory(camera_manager)


find_package(OpenCV REQUIRED)
# 打印 OpenCV 信息（用于调试）
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "OpenCV Include Dir: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV Libraries: ${OpenCV_LIBS}")

add_executable(dvs_rgb_calib dvs_rgb_calib.cpp)
target_link_libraries(dvs_rgb_calib 
    camera_combo_manager
    ${OpenCV_LIBS}
)
target_include_directories(dvs_rgb_calib 
PRIVATE 
    ${CALIB_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}   
)

add_executable(file_reader file_reader.cpp)
target_link_libraries(file_reader 
    camera_combo_manager
    ${OpenCV_LIBS}
)
target_include_directories(file_reader 
PRIVATE 
    ${CALIB_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}   
)

add_executable(dvs_rgb_replay dvs_rgb_replay.cpp)
target_link_libraries(dvs_rgb_replay 
    camera_combo_manager
    ${OpenCV_LIBS}
)
target_include_directories(dvs_rgb_replay 
PRIVATE 
    ${CALIB_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}   
)
#add_executable(dvs_show_corners dvs_show_corners.cpp)
#target_link_libraries(dvs_show_corners 
#    ${SAMPLE_LIBRARIES}
#    camera_combo_manager
#)
#target_include_directories(dvs_show_corners PUBLIC ${CALIB_INCLUDE_DIRS})

#add_executable(blinking_corners blinking_corners.cpp)
#target_link_libraries(blinking_corners 
#    ${SAMPLE_LIBRARIES}
#)
#target_include_directories(blinking_corners PUBLIC ${CALIB_INCLUDE_DIRS})

#add_executable(dvs_single_calib dvs_single_calib.cpp)
#target_link_libraries(dvs_single_calib 
#    ${SAMPLE_LIBRARIES}
#    camera_combo_manager
#)
#target_include_directories(dvs_single_calib PUBLIC ${CALIB_INCLUDE_DIRS})