# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

add_executable(mrob_imu_data_extractor main.c ${CMAKE_CURRENT_BINARY_DIR}/version.rc)

target_link_libraries(mrob_imu_data_extractor PRIVATE
    k4a::k4a
    k4a::k4arecord
    )

# Include ${CMAKE_CURRENT_BINARY_DIR}/version.rc in the target's sources
# to embed version information
set(K4A_FILEDESCRIPTION "Azure Kinect IMU Data Extraction Tool")
set(K4A_ORIGINALFILENAME "mrob_imu_data_extractor.exe")
configure_file(
    ${K4A_VERSION_RC}
    ${CMAKE_CURRENT_BINARY_DIR}/version.rc
    @ONLY
    )

# Setup install
include(GNUInstallDirs)

install(
    TARGETS
        mrob_imu_data_extractor
    RUNTIME DESTINATION
        ${CMAKE_INSTALL_BINDIR}
    COMPONENT
        tools
)

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows")
    install(
        FILES
            $<TARGET_PDB_FILE:mrob_imu_data_extractor>
        DESTINATION
            ${CMAKE_INSTALL_BINDIR}
        COMPONENT
            tools
        OPTIONAL
    )
endif()