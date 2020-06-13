import time

import pytest
import requests
from farm_ng_proto.tractor.v1 import playback_service_pb2

PORT = 9011
BASE_URL = 'http://localhost:' + str(PORT) + '/twirp/farm_ng_proto.tractor.v1.PlaybackService'


def make_request(name, request_proto, response_proto_type):
    response = requests.post(
        BASE_URL + '/' + name,
        data=request_proto.SerializeToString(),
        headers={'Content-Type': 'application/protobuf'},
    )
    assert response.status_code == 200
    response_pb = response_proto_type()
    response_pb.ParseFromString(response.content)
    return response_pb


@pytest.mark.integration
class TestClass:
    def test_list_logs(self):
        response_proto = make_request('ListLogs', playback_service_pb2.ListLogsRequest(), playback_service_pb2.ListLogsResponse)
        print(response_proto)
        assert len(response_proto.log_paths) >= 1
        return response_proto

    def test_open_log(self):
        list_response = self.test_list_logs()
        req = playback_service_pb2.OpenLogRequest(log_path=list_response.log_paths[-1])
        open_response = make_request('OpenLog', req, playback_service_pb2.OpenLogResponse)
        print(open_response)
        return open_response

    def test_play_log(self):
        open_response = self.test_open_log()
        req = playback_service_pb2.PlayRequest(
            start_event=open_response.n_events//2,
            n_events=100,
            playback_rate=playback_service_pb2.PlayRequest.RATE_REALTIME,
            loop=False,
        )

        make_request('Play', req, playback_service_pb2.PlayResponse)
        time.sleep(2)
        make_request('Stop', playback_service_pb2.StopRequest(), playback_service_pb2.StopResponse)
