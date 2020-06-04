from typing import Optional

from dataclasses import dataclass
from farm_ng.tractor.sim.models.duration import Duration
from farm_ng_proto.tractor.v1 import waypoint_pb2
from google.protobuf import wrappers_pb2


@dataclass
class Waypoint:
    """
    Models like this should exist on-robot, separate from their serialized representation.
    Models provide helper functions to (de)serialize to/from the network, database, etc.
    """
    id: int
    lat: float
    lng: float
    angle: float
    delay: Optional[Duration]
    radius: Optional[float]

    @staticmethod
    def fromProto(proto: waypoint_pb2.Waypoint) -> 'Waypoint':
        return Waypoint(
            id=proto.id.value if proto.HasField('id') else None,
            lat=proto.lat,
            lng=proto.lng,
            angle=proto.angle,
            delay=Duration.fromProto(
                proto.delay,
            ) if proto.HasField('delay') else None,
            radius=proto.radius.value if proto.HasField('radius') else None,
        )

    def toProto(self) -> waypoint_pb2.Waypoint:
        return waypoint_pb2.Waypoint(
            id=wrappers_pb2.UInt32Value(
                value=self.id,
            ) if self.id else None,
            lat=self.lat,
            lng=self.lng,
            angle=self.angle,
            delay=self.delay.toProto() if self.delay else None,
            radius=wrappers_pb2.DoubleValue(
                value=self.radius,
            ) if self.radius else None,
        )
