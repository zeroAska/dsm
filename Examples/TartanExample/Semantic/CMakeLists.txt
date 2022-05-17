cmake_minimum_required(VERSION 3.5)

# Set the name of the Program
set(PROGRAM_NAME TartanSemanticExample)

add_executable(${PROGRAM_NAME} main.cpp)

include_directories(${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_BINARY_DIR}
	${CMAKE_SOURCE_DIR}/thirdparty/Sophus
	${EIGEN3_INCLUDE_DIR}
	${QT_VISUALIZER_LIB_INCLUDE_DIR}
	${DSM_LIB_INCLUDE_DIR}
        #${UnifiedCvo_INCLUDE_DIR} 
	)
if (${PROJECT_NAME}_ENABLE_QT) 
  target_link_libraries(${PROGRAM_NAME} ${OpenCV_LIBS} QtVisualizer dsm UnifiedCvo::cvo_gpu_img_lib UnifiedCvo::tartan UnifiedCvo::cvo_utils_lib UnifiedCvo::elas boost_system )
else()
  target_link_libraries(${PROGRAM_NAME} ${OpenCV_LIBS} dsm UnifiedCvo::cvo_gpu_img_lib UnifiedCvo::tartan UnifiedCvo::cvo_utils_lib UnifiedCvo::elas boost_system )
endif()
target_compile_definitions(${PROGRAM_NAME} PUBLIC ${PCL_DEFINITIONS})
set_target_properties(${PROGRAM_NAME} PROPERTIES FOLDER "Examples")
