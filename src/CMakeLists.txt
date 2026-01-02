find_library(GRPC_PROTO_LIB grpc_proto_lib)
if (NOT GRPC_PROTO_LIB)
    message(FATAL_ERROR "Not found GRPC_PROTO Library.")
endif()

find_library(LOGGER_LIB logger_lib)
if (NOT LOGGER_LIB)
    message(FATAL_ERROR "Not found Logger Library.")
endif()

add_executable(fpga_driver  color.cpp
                            console.cpp       
                            fpga_handler.cpp
                            main.cpp
                            motor_fsm.cpp
                            robot_fsm.cpp
                            leg_module.cpp
                            can_motor.cpp
                            can_channel.cpp
                            NiFpga_FPGA_CANBus_4module_v3_steering.c
                            NiFpga.c                       
)
find_package(yaml-cpp REQUIRED)
target_link_libraries(fpga_driver yaml-cpp::yaml-cpp ${CURSES_LIBRARY} -lncurses ${CMAKE_DL_LIBS} ${_CORE_LIBRARIES} ${GRPC_PROTO_LIB} ${LOGGER_LIB} ${LOG_PROTO_LIB})

find_package(Eigen3 REQUIRED)
target_link_libraries(fpga_driver Eigen3::Eigen ${CURSES_LIBRARY} -lncurses ${CMAKE_DL_LIBS} ${_CORE_LIBRARIES} ${GRPC_PROTO_LIB})

