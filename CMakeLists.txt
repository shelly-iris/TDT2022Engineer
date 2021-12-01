cmake_minimum_required(VERSION 3.1)
project(TDTvision2022_engineer)

find_package(OpenCV REQUIRED)
add_definitions(-w)


#设置可执行文件保存路径 
# SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/bin)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Debug)

SET(PROJECT_ROOT_PATH "${PROJECT_SOURCE_DIR}")
SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin/")
SET(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib/")
link_directories(${PROJECT_BINARY_DIR}/lib/)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
# message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/ksconnect)
add_subdirectory(ksconnect)
aux_source_directory(./ksconnect KS_SRCS)

include_directories(${PROJECT_SOURCE_DIR}/light)
add_subdirectory(light)
aux_source_directory(./light LIGHT_SRCS)

include_directories(${PROJECT_SOURCE_DIR}/usart)
add_subdirectory(usart)
aux_source_directory(./usart USART_SRCS)

include_directories(${PROJECT_SOURCE_DIR}/tool)
add_subdirectory(tool)
aux_source_directory(./tool TOOL_SRCS)


# add_executable(../imgui/imgui.cpp ../imgui/imgui_draw.cpp ../imgui/imgui_impl_glfw.cpp)
add_executable(
    TDTvision2022_engineer
    main.cpp
    ${KS_SRCS}
    ${LIGHT_SRCS}
    ${TOOlKIT_SRCS}
    ${USART_SRCS}
    ${TOOL_SRCS}
)

link_directories(${OpenCV_LIBRARY_DIRS})

target_link_libraries(TDTvision2022_engineer PUBLIC ${OpenCV_LIBS} -lpthread -lm)
