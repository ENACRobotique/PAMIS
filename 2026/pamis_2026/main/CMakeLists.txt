idf_build_get_property(target IDF_TARGET)

#############################################################################
# Some magic stuff to generate protobuf files
#############################################################################

# set(PROTO_DIR  ${PROJECT_DIR}/proto)
# set(PROTO_GEN_DIR ${CMAKE_BINARY_DIR}/proto)        # generated files in build/proto/
# set(EMBEDDED_PROTO_DIR ${PROJECT_DIR}/EmbeddedProto)
# file(GLOB PROTO_FILES ${PROTO_DIR}/*.proto)

# if(NOT CMAKE_BUILD_EARLY_EXPANSION)
#     find_program(PROTOC protoc REQUIRED)

#     foreach(proto_file ${PROTO_FILES})
#         get_filename_component(proto_name ${proto_file} NAME_WE)
#         set(output_file ${PROTO_GEN_DIR}/${proto_name}.h)
#         add_custom_command(
#             OUTPUT ${output_file}
#             COMMAND ${CMAKE_COMMAND} -E make_directory ${PROTO_GEN_DIR}
#             COMMAND ${PROTOC}
#                     --plugin=protoc-gen-eams=protoc-gen-eams
#                     -I ${PROTO_DIR}
#                     --eams_out=${PROTO_GEN_DIR}
#                     ${proto_file}
#             DEPENDS ${proto_file}
#             WORKING_DIRECTORY ${EMBEDDED_PROTO_DIR}
#             COMMENT "Generating EmbeddedProto file from ${proto_name}.proto"
#         )
#         list(APPEND GENERATED_HEADERS ${output_file})
#     endforeach()

#     add_custom_target(generate_proto ALL DEPENDS ${GENERATED_HEADERS})
# endif()

#############################################################################

idf_component_register(
    SRCS
        radar_vl53.cpp
        utils.cpp
        imu.cpp
        locomotion.cpp
        SAP_controller.cpp
        sts3032.cpp
        main.cpp
        wifi/websocket_server.cpp
        wifi/wifi_setup.cpp
        wifi/telelogs.cpp
        # wifi/BytesWriteBuffer.cpp
        # wifi/BytesReadBuffer.cpp
        # wifi/protoCom.cpp
        # ${EMBEDDED_PROTO_DIR}/src/Fields.cpp
        # ${EMBEDDED_PROTO_DIR}/src/MessageInterface.cpp
        # ${EMBEDDED_PROTO_DIR}/src/ReadBufferSection.cpp


    PRIV_REQUIRES
        spi_flash
        nvs_flash
        esp_event
        esp_netif
        lwip
        esp_wifi
        esp_http_server
        spiffs
        esp_driver_i2c
    REQUIRES
        driver
        stepper
        vl53l1x
    INCLUDE_DIRS
        .
        wifi
        # ${EMBEDDED_PROTO_DIR}/src
        ${CMAKE_BINARY_DIR}     # for protobuf generated files
)

spiffs_create_partition_image(storage ../data FLASH_IN_PROJECT)
