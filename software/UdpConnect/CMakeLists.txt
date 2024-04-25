cmake_minimum_required(VERSION 2.8.9)
PROJECT(udp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


include_directories(${PROJECT_SOURCE_DIR})
include_directories(/usr/local/include/)
include_directories(/usr/include/ )

# link directories
link_directories("/usr/local/lib")
link_directories("/usr/lib")



file(GLOB MOTIONCONTROL_SRC
     "./*.cpp"
     "./*.c"
)

file(GLOB MAIN
     "./main.cpp"
)


ADD_EXECUTABLE(main ${MAIN} ${MOTIONCONTROL_SRC} )


