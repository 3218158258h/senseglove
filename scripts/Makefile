# Makefile for nova2_sensor_api.so
#
# Builds a thin C-export wrapper around the SenseGlove C++ SDK.
# No ROS2 is needed to build or run the resulting library.
#
# Usage:
#   make          – build  scripts/nova2_sensor_api.so
#   make clean    – remove build artefacts
#
# Prerequisites:
#   g++ (C++17), the SenseGlove SDK libs and headers present at the paths
#   configured below.

# ── Paths ─────────────────────────────────────────────────────────────────────
# Defaults use the SenseGlove-API-master v22 libs (requires only GLIBC_2.17,
# compatible with Ubuntu 22.04).  Override on the command line if needed, e.g.:
#   make SDK_LIB_DIR=/path/to/other/libs
REPO_ROOT   ?= $(shell realpath $(dir $(lastword $(MAKEFILE_LIST)))/..)
SDK_INCLUDE ?= $(REPO_ROOT)/SenseGlove-API-master/include/SenseGlove/Core
SDK_LIB_DIR ?= $(REPO_ROOT)/SenseGlove-API-master/lib/linux/v22/x86-64/release
OUT_DIR     ?= $(dir $(lastword $(MAKEFILE_LIST)))

# ── Compiler flags ───────────────────────────────────────────────────────────
CXX      := g++
CXXFLAGS := -std=c++17 -O2 -fPIC -Wall -Wextra \
            -I$(SDK_INCLUDE)
LDFLAGS  := -shared \
            -L$(SDK_LIB_DIR) \
            -lsgcore -lsgconnect \
            -Wl,-rpath,$(SDK_LIB_DIR) \
            -lpthread

# ── Targets ──────────────────────────────────────────────────────────────────
TARGET := $(OUT_DIR)nova2_sensor_api.so

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OUT_DIR)nova2_sensor_api.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)
	@echo "Built: $@"

clean:
	rm -f $(TARGET)
