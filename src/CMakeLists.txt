file(GLOB SOURCES CONFIGURE_DEPENDS *.cpp)
list(FILTER SOURCES EXCLUDE REGEX "run")
list(FILTER SOURCES EXCLUDE REGEX "main")

add_library(gppc STATIC ${SOURCES} ${HEADERS})
target_include_directories(gppc PUBLIC ${CMAKE_SOURCE_DIR}/include PUBLIC ${CMAKE_SOURCE_DIR}/lib)

add_executable(run run.cpp)
target_link_libraries(run PRIVATE gppc lib)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE gppc lib)

file(COPY ${CMAKE_SOURCE_DIR}/data DESTINATION ${CMAKE_BINARY_DIR}/src)
file(COPY ${CMAKE_SOURCE_DIR}/benchmarks/grid-maps/iron-harvest DESTINATION ${CMAKE_BINARY_DIR}/src)
