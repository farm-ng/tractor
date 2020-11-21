import logging
import os
import pathlib

import google.protobuf.json_format as json_format

from farm_ng.core.resource_pb2 import Bucket

logger = logging.getLogger('blobstore')
logger.setLevel(logging.INFO)


class InvalidBucketException(Exception):
    pass


class Blobstore:
    def __init__(self):
        self.root = os.getenv('BLOBSTORE_ROOT')
        if (not self.root):
            raise Exception('BLOBSTORE_ROOT not set.')

    def read_protobuf_from_json_file(self, path, message):
        self._check_valid_path(path)
        with open(os.path.join(self.root, path)) as f:
            json_format.Parse(f.read(), message)

    def read_protobuf_from_binary_file(self, path, message):
        self._check_valid_path(path)
        with open(os.path.join(self.root, path), 'rb') as f:
            message.ParseFromString(f.read())

    def bucket_relative_path(self, bucket_id):
        name = Bucket.Name(bucket_id)
        return name[len('BUCKET_'):].lower()

    def _check_valid_path(self, path):
        valid_buckets = [self.bucket_relative_path(id) for id in Bucket.values()]
        target_bucket = pathlib.Path(path).parts[0]
        if target_bucket not in valid_buckets:
            raise InvalidBucketException(f'Invalid bucket: {target_bucket}')

    def _write_protobuf_to_json_file(self, path, message):
        raise NotImplementedError()

    def _write_protobuf_to_binary_file(self, path, message):
        raise NotImplementedError()

    def write_protobuf_as_resource(self, path, message, serialization='json'):
        raise NotImplementedError()

    def read_protobuf_from_resource(self, resource):
        raise NotImplementedError()
