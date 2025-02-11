# 
# This file is part of DSM.
# 
# Copyright (C) 2019 CEIT (Universidad de Navarra) and Universidad de Zaragoza
# Developed by Jon Zubizarreta,
# for more information see <https://github.com/jzubizarreta/dsm>.
# If you use this code, please cite the respective publications as
# listed on the above website.
# 
# DSM is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# DSM is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with DSM. If not, see <http://www.gnu.org/licenses/>.
# 

cmake_minimum_required(VERSION 3.5)

# Set the name of the Program
set(PROGRAM_NAME TumExample)


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
target_link_libraries(${PROGRAM_NAME} ${OpenCV_LIBS} QtVisualizer dsm UnifiedCvo::cvo_gpu_img_lib UnifiedCvo::tum UnifiedCvo::cvo_utils_lib UnifiedCvo::elas boost_system )
target_compile_definitions(${PROGRAM_NAME} PUBLIC ${PCL_DEFINITIONS})
set_target_properties(${PROGRAM_NAME} PROPERTIES FOLDER "Examples")
endif()
add_executable(${PROGRAM_NAME}_no_qt main_no_qt.cpp)
target_link_libraries(${PROGRAM_NAME}_no_qt ${OpenCV_LIBS} dsm UnifiedCvo::cvo_gpu_img_lib UnifiedCvo::tum UnifiedCvo::cvo_utils_lib UnifiedCvo::elas boost_system )
target_compile_definitions(${PROGRAM_NAME}_no_qt PUBLIC ${PCL_DEFINITIONS})
set_target_properties(${PROGRAM_NAME}_no_qt PROPERTIES FOLDER "Examples")
