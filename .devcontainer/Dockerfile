###############################################################################
# ESP-IDF Dev-Container base                                                  #
# • Uses Espressif’s official image so all tools are already pre-installed    #
# • Adds a few extras & ensures every shell sources export.sh automatically   #
###############################################################################
ARG DOCKER_TAG=latest
FROM docker.io/espressif/idf:${DOCKER_TAG}

# ─────────────────────────────────────────────────────────────────────────────
# Locale & common build utilities
# ─────────────────────────────────────────────────────────────────────────────
ENV LC_ALL=C.UTF-8 \
    LANG=C.UTF-8

RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends \
    udev           \
    cmake          \
    ninja-build    \
    git            \
    curl           \
    build-essential \
    gdb-multiarch  && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ─────────────────────────────────────────────────────────────────────────────
# Ensure EVERY interactive shell gets ESP-IDF vars (root & esp user)
# ─────────────────────────────────────────────────────────────────────────────
RUN echo 'source /opt/esp/idf/export.sh' >> /root/.bashrc

# ─────────────────────────────────────────────────────────────────────────────
# Default entrypoint stays unchanged – it already sets the IDF env
# ─────────────────────────────────────────────────────────────────────────────
ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]
CMD [ "bash" ]
