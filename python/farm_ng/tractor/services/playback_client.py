import time

import requests
from farm_ng_proto.tractor.v1 import playback_service_pb2
import argparse
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


def main():
    parser = argparse.ArgumentParser()
    args = parser.parse_args()
    list_response = make_request('ListLogs', playback_service_pb2.ListLogsRequest(), playback_service_pb2.ListLogsResponse)
    print(list_response)
    req = playback_service_pb2.OpenLogRequest(log_path=list_response.log_paths[-1])
    open_response = make_request('OpenLog', req, playback_service_pb2.OpenLogResponse)
    print(open_response)
    req = playback_service_pb2.PlayRequest(
        playback_rate=playback_service_pb2.PlayRequest.RATE_ASAP,
        loop=False,
    )
    play_response = make_request('Play', req, playback_service_pb2.PlayResponse)
    print(play_response)
    

if __name__ == "__main__":
    main()
