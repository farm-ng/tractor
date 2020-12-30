FROM farmng/devel@sha256:880930a814388b9ab738fa56114f56ed8158be1e503abfff8ffaac09710df060

WORKDIR /workspace/farm_ng
RUN export FARM_NG_ROOT=/workspace/farm_ng

# Build first-party c++
COPY CMakeLists.txt .
COPY cmake cmake
COPY doc doc
COPY modules modules
COPY setup.bash .
COPY env.sh .
COPY third_party third_party


SHELL ["/bin/bash", "-c"]

ARG PARALLEL=1
RUN . setup.bash && which protoc-gen-ts_proto
RUN . setup.bash && \
  mkdir -p build && \
  cd build && \
   /workspace/farm_ng/env.sh cmake -DCMAKE_PREFIX_PATH=/farm_ng/env -DBUILD_DOCS=TRUE -DCMAKE_BUILD_TYPE=Release .. && \
     make docs && \
  make -j$PARALLEL

RUN cd modules/frontend/frontend && yarn && FARM_NG_ROOT=/workspace/farm_ng yarn build && \
    cp -rT dist /workspace/farm_ng/build/frontend

RUN ./env.sh make -C modules/frontend/go/webrtc
