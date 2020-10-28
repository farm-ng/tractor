# Build on top of cpp image because programd depends on cpp binaries
FROM farmng/cpp

WORKDIR /farm_ng

# Install python deps in a virtualenv
COPY env.sh requirements.txt ./
RUN . ./env/bin/activate && pip install -r ./requirements.txt

# Build protos
COPY python python
COPY protos /protos
RUN protoc \
  --proto_path=/protos \
  --python_out=python/genproto \
  /protos/farm_ng_proto/tractor/v1/*.proto

ENTRYPOINT ["./env.sh", "python", "-m"]
CMD ["farm_ng.ipc"]

# TODO(isherman): Reduce size of final image with multi-stage build
# https://pythonspeed.com/articles/smaller-python-docker-images/
