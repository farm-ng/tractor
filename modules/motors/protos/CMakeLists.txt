farm_ng_add_protobufs(farm_ng_motors_protobuf
  PROTO_FILES
  ./farm_ng/motors/motor_config.proto
  ./farm_ng/motors/motor_vesc.proto
  ./farm_ng/motors/motor_odrive.proto
  DEPENDENCIES
  farm_ng_core_protobuf
  )
