ARG DOCKER_TAG=latest
FROM espressif/idf:${DOCKER_TAG}

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

RUN apt-get update -y && apt-get install udev -y

# Install required system packages
RUN apt-get update && apt-get install -y \
    cmake \
    ninja-build \
    git \
    curl \
    python3-pip \
    build-essential \
    gdb-multiarch \
    && apt-get clean

RUN echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> ~/.bashrc

ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

CMD ["/bin/bash", "-c"]