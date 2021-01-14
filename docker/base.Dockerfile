FROM farmng/devel@sha256:ae2c7f801fcd8ffda5ca1e3f24321f3eef5962b81fb1d57026816aca22e270e7

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

# RUN ./env.sh make cpp  && ./env.sh make webservices
