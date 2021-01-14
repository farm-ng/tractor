FROM farmng/devel@sha256:0be3c6a66403711dd76fac557d5c73c4fb626bcd74795a106ab65e1b251e0c6d

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
