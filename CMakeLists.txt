# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
#
cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

string(REGEX REPLACE ".*/(.*)" "\\1" CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR})
# 定义工程名称，指定语言（默认支持所有）
# 该指令隐式定义了两个变量：PROJECT_SOURCE_DIR(指代工程路径)和PROJECT_BINARY_DIR(指代编译路径./build)
# 通过$()引用变量
# 
PROJECT(${CURRENT_FOLDER})

# path
# REQUIRED:如果未找到任何内容，则停止处理并显示错误消息
enable_language(CUDA)
find_package(CUDA REQUIRED)
find_package(PCL REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(Torch REQUIRED)
# NO_MODULE PATHS "E:/libtorch" NO_DEFAULT_PATH

# 变量
set(Python3_ROOT_DIRS "E:/Users/qxc/AppData/Local/Programs/Python/Python38")
set(LMI_ROOT_DIRS "E:/LMI/14400-6.0.21.13_SOFTWARE_GO_SDK/GO_SDK")

# 头文件搜索路径
include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${Python3_ROOT_DIRS}/Lib/site-packages/numpy/core/include
  "./include/"
  "E:/Inovance 1.7.5/Release"
  ${LMI_ROOT_DIRS}/Gocator/GoSdk
  ${LMI_ROOT_DIRS}/Platform/kApi
)

# .lib文件搜索路径
link_directories(${PCL_LIBRARY_DIRS})  
            
add_definitions(${PCL_DEFINITIONS})                              

# file
#file(GLOB CU_FILES "./src/*.cu")
file(GLOB CPP_FILES "./src/*.cpp")
file(GLOB LIB_FILES "${LMI_ROOT_DIRS}/lib/win64/*.lib" "E:/Inovance 1.7.5/Release/*.lib")

# target
# 参数之间用空格或者分号隔开
# 一个文件名为 hello的可执行文件，相关的源文件是file中定义的源文件列表，
add_executable(${PROJECT_NAME} ${CPP_FILES} ${CU_FILES})

# .lib文件
target_link_libraries(${PROJECT_NAME} 
                      ${PCL_LIBRARIES} 
                      ${Python3_LIBRARIES}
                      ${TORCH_LIBRARIES}
                      ${LIB_FILES})  
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

MESSAGE("${Python3_INCLUDE_DIRS}")
MESSAGE("${TORCH_INCLUDE_DIRS}")
MESSAGE("this is ${LIB_FILES}")
