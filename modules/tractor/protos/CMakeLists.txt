farm_ng_add_protobufs(farm_ng_tractor_protobuf
  PROTO_FILES
  ./farm_ng/tractor/steering.proto
  ./farm_ng/tractor/tractor.proto
  DEPENDENCIES
  farm_ng_perception_protobuf
  farm_ng_motors_protobuf
  )
