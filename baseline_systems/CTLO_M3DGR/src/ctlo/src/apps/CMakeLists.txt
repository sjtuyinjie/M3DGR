add_executable(${PROJECT_NAME} main.cpp  
                               common_utility.cpp)
target_link_libraries(${PROJECT_NAME} 
                         ${PROJECT_NAME}.common
                         ${PROJECT_NAME}.pre
                         ${PROJECT_NAME}.liw
                         ${third_party_libs}
                        )
