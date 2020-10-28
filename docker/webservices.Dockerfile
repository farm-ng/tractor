# Build backend
FROM golang:1.15-alpine AS backend_build

# Install system dependencies
RUN apk add --no-cache git protobuf protobuf-dev

# Copy source
COPY go /go
COPY protos /protos
COPY third_party/api-common-protos/google /protos/google

# Build protos
RUN go get -u github.com/golang/protobuf/protoc-gen-go
RUN go get -u github.com/twitchtv/twirp/protoc-gen-twirp
RUN protoc \
  --proto_path=/protos \
  --go_out=module=github.com/farm_ng/genproto:/go/genproto \
  --twirp_out=paths=source_relative:/go/genproto \
  /protos/farm_ng_proto/tractor/v1/*.proto

# Twirp doesn't yet provide the 'module' flag to output generated code in a structure compatible
# with Go Modules (https://github.com/twitchtv/twirp/issues/226), so clean it up manually.
RUN find /go/genproto/farm_ng_proto -name *.twirp.go -print0 -exec mv {} /go/genproto \;
RUN rm -rf /go/genproto/farm_ng_proto

# Build the server as a static binary
WORKDIR /go/webrtc
RUN CGO_ENABLED=0 go build -o /bin/proxy-server cmd/proxy-server/main.go

# Build frontend
FROM node:12-alpine AS frontend_build

WORKDIR /farm_ng

RUN apk add --no-cache git protobuf protobuf-dev
RUN npm install -g long ts-proto@^1.37.0
COPY app/frontend app/frontend
COPY protos /protos
RUN protoc \
  --proto_path=/protos \
  --ts_proto_out=app/frontend/genproto \
  --ts_proto_opt=forceLong=long \
  /protos/farm_ng_proto/tractor/v1/*.proto

RUN	cd app/frontend && yarn && yarn build

# Copy binary into a single layer image
FROM scratch
COPY --from=backend_build /bin/proxy-server /bin/proxy-server
COPY --from=frontend_build /farm_ng/app/frontend/dist /farm_ng/build/frontend
ENTRYPOINT ["/bin/proxy-server"]
