ARG FARM_NG_DEVEL_IMAGE=farmng/devel:latest
FROM $FARM_NG_DEVEL_IMAGE

WORKDIR $FARM_NG_ROOT

# Build first-party c++
COPY version.sh .
COPY Makefile .
COPY CMakeLists.txt .
COPY cmake cmake
COPY doc doc
COPY modules modules
COPY third_party third_party

RUN make doc cpp webservices
