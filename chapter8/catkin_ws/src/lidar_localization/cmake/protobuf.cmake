include(FindProtobuf)
find_package (Protobuf REQUIRED "3.14.0")

include_directories("/home/bobododo/thridparty/protobuf-3.14.0/src/")
list(APPEND ALL_TARGET_LIBRARIES ${Protobuf_LIBRARIES})
