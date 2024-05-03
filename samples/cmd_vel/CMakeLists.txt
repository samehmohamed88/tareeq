add_executable(sabertooth_tests
        sabertooth_tests.cpp)

target_link_libraries(sabertooth_tests
        ${ACTUATOR_LIBRARY_NAME}
        ${IO_LIBRARY_NAME}
)

install(TARGETS sabertooth_tests
        DESTINATION lib/${PROJECT_NAME})
