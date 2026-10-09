file(READ "${CMAKE_CURRENT_SOURCE_DIR}/dyt.xml" DYT_XML_CONTENT)
configure_file(dyt_xml.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/dyt_xml.hpp" @ONLY)

target_sources(mavsdk PRIVATE dyt.cpp)

target_include_directories(mavsdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/mavsdk>
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
)

install(FILES include/plugins/dyt/dyt.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mavsdk/plugins/dyt)
install(FILES dyt.xml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/mavsdk/mavlink)

list(APPEND UNIT_TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/dyt_test.cpp)
set(UNIT_TEST_SOURCES ${UNIT_TEST_SOURCES} PARENT_SCOPE)
