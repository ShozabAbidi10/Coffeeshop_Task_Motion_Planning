project(Test)


set( CMAKE_MODULE_PATH    ${CMAKE_MODULE_PATH}
                          ${CMAKE_CURRENT_SOURCE_DIR}
                          ${CMAKE_CURRENT_SOURCE_DIR}/cmake
                          )


set(CMAKE_CXX_FLAGS "-Wall -std=gnu++11 ${CMAKE_CXX_FLAGS}")


find_package(Armadillo REQUIRED)

include_directories(${ARMADILLO_INCLUDE_DIRS})
add_library(Visits SHARED VisitSolver.cpp ExternalSolver.cpp)
add_executable(VisitMain VisitSolver.cpp ExternalSolver.cpp main.cpp)
target_link_libraries(VisitMain ${ARMADILLO_LIBRARIES})
target_link_libraries(Visits ${ARMADILLO_LIBRARIES})