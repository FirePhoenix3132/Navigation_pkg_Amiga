farm_ng_add_protobufs(farm_ng_amiga_proto_defs
    NAMESPACE farm_ng_amiga
    INCLUDE_DIRS
      ${farm_ng_core_PROTO_DIR}
    PROTO_FILES
        farm_ng/amiga/amiga.proto
        farm_ng/canbus/amiga_v6.proto
        farm_ng/canbus/canbus.proto
        farm_ng/control/control.proto
        farm_ng/gps/gps.proto
        farm_ng/imu/imu.proto
        farm_ng/filter/filter.proto
        farm_ng/oak/oak.proto
    DEPENDENCIES
        farm_ng_core::farm_ng_core_proto_defs
)
