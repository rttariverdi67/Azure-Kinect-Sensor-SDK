# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

find_package(OpenCV)


add_executable(mrob_images_extractor
    main.cpp
    transformation_helpers.cpp
    transformation_helpers.h)

target_link_libraries(mrob_images_extractor PRIVATE
    k4a::k4a
    k4a::k4arecord
    libjpeg-turbo::libjpeg-turbo
    ${OpenCV_LIBS}
    )

# Include ${CMAKE_CURRENT_BINARY_DIR}/version.rc in the target's sources
# to embed version information
set(K4A_FILEDESCRIPTION "Azure Kinect Color and Depth Images Extraction Tool where Depth is Projected to Color")
set(K4A_ORIGINALFILENAME "mrob_images_extractor.exe")
configure_file(
    ${K4A_VERSION_RC}
    ${CMAKE_CURRENT_BINARY_DIR}/version.rc
    @ONLY
    )

# Setup install
include(GNUInstallDirs)

install(
    TARGETS
        mrob_images_extractor
    RUNTIME DESTINATION
        ${CMAKE_INSTALL_BINDIR}
    COMPONENT
        tools
)

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows")
    install(
        FILES
            $<TARGET_PDB_FILE:mrob_images_extractor>
        DESTINATION
            ${CMAKE_INSTALL_BINDIR}
        COMPONENT
            tools
        OPTIONAL
    )
endif()