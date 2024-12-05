cmake_minimum_required(VERSION 2.8.9)
PROJECT(robot)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
add_compile_options(-Wuninitialized)

#include directories
include_directories(${PROJECT_SOURCE_DIR}/include/dynamixel)
include_directories(${PROJECT_SOURCE_DIR}/include/INA219)
include_directories(${PROJECT_SOURCE_DIR}/include/SVnPUMPnIMU)
include_directories(${PROJECT_SOURCE_DIR}/include/MotionControl)
include_directories(/usr/local/include/)
include_directories(/usr/include/ )

# link directories
link_directories("/usr/local/lib")
link_directories("/usr/lib")

#identify the processor, mark "DYNAMIXELLIB"
message("CMAKE_HOST_SYSTEM_NAME: ${CMAKE_HOST_SYSTEM_NAME} ")
message("CMAKE_HOST_SYSTEM_PROCESSOR: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
if(CMAKE_HOST_SYSTEM_NAME MATCHES "Darwin")
set( DYNAMIXELLIB "dxl_mac_cpp")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
set( DYNAMIXELLIB "dxl_x64_cpp")
else()
set( DYNAMIXELLIB "dxl_sbc_cpp")
endif()

file(GLOB INA219_SRC
     "./src/INA219/*.cpp"
     "./src/INA219/*.c"
)

file(GLOB DYNAMIXEL_SRC
	"./src/dynamixel/*.cpp"
	"./src/dynamixel/*.c"
	)

file(GLOB SVPUMPIMU_SRC
     "./src/SVnPUMPnIMU/*.cpp"
     "./src/SVnPUMPnIMU/*.c"
)

file(GLOB MOTIONCONTROL_SRC
     "./src/MotionControl/*.cpp"
     "./src/MotionControl/*.c"
)

file(GLOB MAIN
     "./src/main.cpp"
)

file(GLOB TEST
	"./src/test.cpp"
)

file(GLOB VIBRA
     "./src/vibra.cpp"
)
file(GLOB IMUTEST
     "./src/imutest.cpp"
)
# ADD_EXECUTABLE(main ${MAIN} ${MOTIONCONTROL_SRC} )
# target_link_libraries(main /usr/local/lib/libqpOASES.a)

  ADD_EXECUTABLE(main ${MAIN} ${SVPUMPIMU_SRC} ${INA219_SRC} ${MOTIONCONTROL_SRC} ${DYNAMIXEL_SRC}) 
  target_link_libraries(main -lwiringPi ${DYNAMIXELLIB} /usr/local/lib/libqpOASES.a -lpthread)

# ADD_EXECUTABLE(test ${TEST} ${SVPUMPIMU_SRC} ${INA219_SRC} ${DYNAMIXEL_SRC} ${MOTIONCONTROL_SRC}) 
# target_link_libraries(test -lwiringPi ${DYNAMIXELLIB} /usr/local/lib/libqpOASES.a -lpthread)

#  ADD_EXECUTABLE(test ${TEST} ${SVPUMPIMU_SRC} ${INA219_SRC}) 
#  target_link_libraries(test -lwiringPi)

# ADD_EXECUTABLE(vibra ${VIBRA} ${SVPUMPIMU_SRC} ${INA219_SRC} ${MOTIONCONTROL_SRC} ${DYNAMIXEL_SRC}) 
# target_link_libraries(vibra -lwiringPi ${DYNAMIXELLIB} /usr/local/lib/libqpOASES.a -lpthread)

# ADD_EXECUTABLE(imutest ${IMUTEST} ${SVPUMPIMU_SRC} ${INA219_SRC} ${MOTIONCONTROL_SRC} ${DYNAMIXEL_SRC}) 
# target_link_libraries(imutest -lwiringPi ${DYNAMIXELLIB} /usr/local/lib/libqpOASES.a -lpthread)