# Build backend
FROM golang:1.15-alpine AS build

# Install system dependencies
RUN apk add --no-cache alpine-sdk bash cmake git npm protobuf protobuf-dev yarn

WORKDIR /farm_ng

# Copy source
COPY CMakeLists.txt .
COPY cmake cmake
COPY modules modules
COPY doc doc
COPY third_party/api-common-protos/google third_party/api-common-protos/google

# Build protos
RUN go get -u github.com/golang/protobuf/protoc-gen-go
RUN go get -u github.com/twitchtv/twirp/protoc-gen-twirp
RUN npm install -g long ts-proto@^1.37.0
# TODO(isherman): These DISABLE flags are super brittle; clean up.
RUN	mkdir -p build && \
  cd build && \
  cmake -DBUILD_ONLY_PROTO=TRUE -DDISABLE_PROTOC_cpp=TRUE -DDISABLE_PROTOC_python=true -DCMAKE_PREFIX_PATH=`pwd`/../env -DCMAKE_BUILD_TYPE=Release .. && \
  make -j`nproc --ignore=1`

# Build the server as a static binary
WORKDIR /farm_ng/modules/frontend/go/webrtc
RUN CGO_ENABLED=0 go build -o /bin/proxy-server cmd/proxy-server/main.go

# Build the web application
WORKDIR /farm_ng/modules/frontend/frontend
RUN yarn
RUN npm rebuild node-sass
RUN FARM_NG_ROOT=/farm_ng yarn build

# Copy binary into a single layer image
FROM scratch
COPY --from=build /bin/proxy-server /bin/proxy-server
COPY --from=build /farm_ng/modules/frontend/frontend/dist /farm_ng/build/frontend
ENTRYPOINT ["/bin/proxy-server"]
