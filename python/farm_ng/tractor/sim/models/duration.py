from dataclasses import dataclass
from google.protobuf import duration_pb2


@dataclass
class Duration:
    seconds: int
    nanos: int

    @staticmethod
    def fromProto(proto: duration_pb2.Duration) -> 'Duration':
        return Duration(seconds=proto.seconds, nanos=proto.nanos)

    def toProto(self) -> duration_pb2.Duration:
        return duration_pb2.Duration(seconds=self.seconds, nanos=self.nanos)
