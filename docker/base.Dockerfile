FROM farmng/devel@sha256:15c31ddbeb3649575de3dfbc4c3360c1b5af04e04fbc275fd7c9de4cde60b89d

WORKDIR /workspace/farm_ng
RUN export FARM_NG_ROOT=/workspace/farm_ng

# Build first-party c++
COPY Makefile .
COPY CMakeLists.txt .
COPY cmake cmake
COPY doc doc
COPY modules modules
COPY setup.bash .
COPY env.sh .
COPY third_party third_party


SHELL ["/bin/bash", "-c"]

RUN ./env.sh make doc cpp webservices
