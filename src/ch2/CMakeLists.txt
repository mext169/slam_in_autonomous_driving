add_executable(motion motion.cc)
target_link_libraries(motion
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )

add_executable(motion_assignment motion_assignment.cc)
target_link_libraries(motion_assignment
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )