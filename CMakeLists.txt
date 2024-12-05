cmake_minimum_required(VERSION 3.0)
project(Cluster)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CONFIGURATION_TYPES Debug RelWithDebInfo Release)
set(CMAKE_C_FLAGS_RELEASE  "${CMAKE_C_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -pg -march=native")
set(CMAKE_CXX_FLAGS_RELEASE   "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG    "${CMAKE_CXX_FLAGS_DEBUG} -Wall -pg -march=native")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)


include_directories(
                    ${PROJECT_SOURCE_DIR}/src
                    ${Sophus_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS} 
                    ${YAML_CPP_INCLUDE_DIR} )

find_package(OpenCV REQUIRED)
# find_package(faiss REQUIRED)
# find_package(OpenMP REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${FAISS_INCLUDE_DIRS})
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)


find_package(PCL REQUIRED)
add_executable(cluster_view src/main.cpp src/SemanticCluster.cpp)
target_link_libraries(cluster_view 
            ${PCL_LIBRARIES}
            ${OpenCV_LIBS}
            ${YAML_CPP_LIBRARIES}
            ${EIGEN3_LIBRARIES}
            )

