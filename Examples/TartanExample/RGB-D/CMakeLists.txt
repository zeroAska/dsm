cmake_minimum_required(VERSION 3.5)

# Set the name of the Program
set(PROGRAM_NAME TartanRGBDExample)



include_directories(${CMAKE_CURRENT_BINARY_DIR}
	${CMAKE_BINARY_DIR}
	${CMAKE_SOURCE_DIR}/thirdparty/Sophus
	${EIGEN3_INCLUDE_DIR}
	${QT_VISUALIZER_LIB_INCLUDE_DIR}
	${DSM_LIB_INCLUDE_DIR}
        #${UnifiedCvo_INCLUDE_DIR} 
	)
if (${PROJECT_NAME}_ENABLE_QT)
add_executable(${PROGRAM_NAME} main.cpp)        
target_link_libraries(${PROGRAM_NAME} ${OpenCV_LIBS} QtVisualizer dsm UnifiedCvo::cvo_gpu_img_lib UnifiedCvo::tartan UnifiedCvo::cvo_utils_lib UnifiedCvo::elas boost_system )
target_compile_definitions(${PROGRAM_NAME} PUBLIC ${PCL_DEFINITIONS})
set_target_properties(${PROGRAM_NAME} PROPERTIES FOLDER "Examples")
endif()
#else ()
#  message("Compile without qt")
add_executable(${PROGRAM_NAME}_no_qt main_no_qt.cpp)
target_link_libraries(${PROGRAM_NAME}_no_qt ${OpenCV_LIBS} dsm UnifiedCvo::cvo_gpu_img_lib UnifiedCvo::tartan UnifiedCvo::cvo_utils_lib UnifiedCvo::elas boost_system )
#endif()
target_compile_definitions(${PROGRAM_NAME}_no_qt PUBLIC ${PCL_DEFINITIONS})
set_target_properties(${PROGRAM_NAME}_no_qt PROPERTIES FOLDER "Examples")


add_executable(${PROGRAM_NAME}_local_mapping main_local_mapping_edge_semantic.cpp)
target_link_libraries(${PROGRAM_NAME}_local_mapping ${OpenCV_LIBS} dsm UnifiedCvo::cvo_gpu_img_lib UnifiedCvo::tartan UnifiedCvo::cvo_utils_lib UnifiedCvo::elas boost_system )
target_compile_definitions(${PROGRAM_NAME}_local_mapping PUBLIC ${PCL_DEFINITIONS})
set_target_properties(${PROGRAM_NAME}_local_mapping PROPERTIES FOLDER "Examples")
