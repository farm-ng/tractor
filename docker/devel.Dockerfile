FROM ubuntu:18.04

WORKDIR /farm_ng
RUN export FARM_NG_ROOT=/farm_ng

COPY bootstrap-apt.sh ./
RUN ./bootstrap-apt.sh

COPY bootstrap-venv.sh requirements.txt ./
RUN ./bootstrap-venv.sh

# Install third-party python
COPY setup.bash env.sh ./

# Build third-party c++
COPY third_party third_party

#RUN cd third_party && ./install.sh
