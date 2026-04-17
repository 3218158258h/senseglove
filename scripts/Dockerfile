# Dockerfile – nova2_revo2_bridge_standalone
#
# Uses Ubuntu 22.04 with the SenseGlove-API-master v22 pre-built libs.
# These libs require only GLIBC_2.17, which Ubuntu 22.04 (glibc 2.35) satisfies.
#
# Build (from repo root):
#   docker build -t nova2-bridge -f scripts/Dockerfile .
#
# Run (see run_docker.sh for the full command):
#   docker run --rm -it --privileged --network host nova2-bridge

FROM ubuntu:22.04

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

# Copy the SenseGlove-API-master (headers + v22 prebuilt libs)
COPY SenseGlove-API-master/ ./SenseGlove-API-master/

# Copy the scripts directory
COPY scripts/ ./scripts/

# ── Build the C wrapper ────────────────────────────────────────────────────────
RUN make -C scripts/ OUT_DIR=/app/scripts/

# Add the SDK libs to the dynamic linker path
ENV LD_LIBRARY_PATH=/app/SenseGlove-API-master/lib/linux/v22/x86-64/release:${LD_LIBRARY_PATH}

# ── Entry point ────────────────────────────────────────────────────────────────
CMD ["python3", "scripts/nova2_revo2_bridge_standalone.py"]
