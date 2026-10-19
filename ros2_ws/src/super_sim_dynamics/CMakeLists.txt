cmake_minimum_required(VERSION 3.8)
project(super_sim_dynamics)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(super_sim_dynamics)

install(
    PROGRAMS
        super_sim_dynamics/vehicle_node.py
        super_sim_dynamics/teleop_node.py
        super_sim_dynamics/autonomy_node.py
    DESTINATION lib/super_sim_dynamics
)

install(
    DIRECTORY launch config
    DESTINATION share/super_sim_dynamics
)

# Keep simulator assets relocatable in the installed package.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../../models/"
    DESTINATION share/${PROJECT_NAME}/models PATTERN "*.bak*" EXCLUDE)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../../worlds/"
    DESTINATION share/${PROJECT_NAME}/worlds PATTERN "*.bak*" EXCLUDE)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(physics test/test_14dof_physics.py
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
    ament_add_pytest_test(autonomy test/test_autonomy.py
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
    ament_add_pytest_test(autonomy_ros test/test_autonomy_ros.py
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()
ament_package()
