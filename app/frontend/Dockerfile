FROM balenalib/jetson-xavier-debian:latest

RUN [ "cross-build-start" ]
RUN apt-get update && apt-get install -y python3-setuptools python3-pip
RUN pip3 install tornado
COPY ./ /app
RUN [ "cross-build-end" ]
CMD ["python3", "/app/main.py"]