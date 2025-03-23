file(GLOB SOURCES CONFIGURE_DEPENDS *.cpp)
list(FILTER SOURCES EXCLUDE REGEX "run")
list(FILTER SOURCES EXCLUDE REGEX "main")

add_executable(run run.cpp ${SOURCES} ${HEADERS})
target_include_directories(run PRIVATE ${CMAKE_SOURCE_DIR}/include PRIVATE ${CMAKE_SOURCE_DIR}/lib)
target_link_libraries(run PRIVATE lib)

add_executable(main main.cpp ${SOURCES} ${HEADERS})
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/include PRIVATE ${CMAKE_SOURCE_DIR}/lib)
target_link_libraries(main PRIVATE lib)

file(COPY ${CMAKE_SOURCE_DIR}/data DESTINATION ${CMAKE_BINARY_DIR}/src)
