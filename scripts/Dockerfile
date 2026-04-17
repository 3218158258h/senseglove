# Dockerfile – nova2_revo2_bridge_standalone
#
# Ubuntu 24.04 ships with glibc 2.39, which satisfies the minimum requirement
# of the pre-built SenseGlove SDK libraries (libsgcore.so requires GLIBC_2.38,
# libsgconnect.so requires GLIBC_2.39).
#
# Build:
#   docker build -t nova2-bridge scripts/
#
# Run (see run_docker.sh for the full command):
#   docker run --rm -it --privileged --network host nova2-bridge

FROM ubuntu:24.04

# ── System packages ────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        g++ \
        make \
        python3 \
        python3-pip \
        libstdc++6 \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ── Working directory ──────────────────────────────────────────────────────────
WORKDIR /app

# Copy the entire senseglove_api (headers + prebuilt libs)
COPY senseglove_api/ ./senseglove_api/

# Copy the scripts directory
COPY scripts/ ./scripts/

# ── Build the C wrapper ────────────────────────────────────────────────────────
RUN make -C scripts/ \
        SDK_INCLUDE=/app/senseglove_api/include/SenseGlove/Core \
        SDK_LIB_DIR=/app/senseglove_api/lib/Ubuntu/x86-64/release \
        OUT_DIR=/app/scripts/

# Add the SDK libs to the dynamic linker path
ENV LD_LIBRARY_PATH=/app/senseglove_api/lib/Ubuntu/x86-64/release:${LD_LIBRARY_PATH}

# ── Revo2 Python SDK (install from the project if bundled, else pip) ───────────
# Uncomment and adjust if the Revo2 SDK ships as a local wheel or source dir:
# COPY stark-serialport-example-main/ ./stark-serialport-example-main/
# RUN pip3 install --no-cache-dir ./stark-serialport-example-main/

# ── Entry point ────────────────────────────────────────────────────────────────
CMD ["python3", "scripts/nova2_revo2_bridge_standalone.py"]
