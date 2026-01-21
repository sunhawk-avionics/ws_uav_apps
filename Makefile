# ============================================================
# Workspace Makefile for ROS 2 (Humble) + colcon
# - Always sources Tools/setup_env.sh before build/run
# - Debug build is breakpoint-friendly (-O0, -g3)
# ============================================================

SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c

WS_DIR  := $(shell pwd)
TOOLS_DIR := $(WS_DIR)/Tools
ENV_SCRIPT := $(TOOLS_DIR)/setup_env.sh

# Helper: run any command inside a login shell that sources env first.
# Note: "source" affects only the subshell, which is exactly what we want.
define WITH_ENV
set +u; source "$(ENV_SCRIPT)" >/dev/null; set -u; $(1)
endef

.PHONY: help env check-env format \
        debug release \
        clean debug-clean release-clean \
        list-bin

help:
	@echo "Targets:"
	@echo "  make env                - print current ROS2 environment order"
	@echo "  make debug              - Debug build (-O0 -g3), breakpoint-friendly"
	@echo "  make release            - Release build"
	@echo "  make clean              - remove build/install/log"
	@echo "  make debug-clean        - clean + debug build"
	@echo "  make release-clean      - clean + release build"
	@echo "  make list-bin           - list installed executables for sunhawk_debug"
	@echo "  make format             - run astyle formatter (if script exists)"

env: check-env
	@$(call WITH_ENV, echo "ROS_DISTRO=$$ROS_DISTRO"; echo "$$AMENT_PREFIX_PATH" | tr ":" "\n" | nl -ba)

check-env:
	@if [ ! -f "$(ENV_SCRIPT)" ]; then \
		echo "[ERROR] Missing env script: $(ENV_SCRIPT)"; \
		echo "        Please create it (Tools/setup_env.sh)"; \
		exit 1; \
	fi

format:
	@if [ -x "$(TOOLS_DIR)/astyle/check_code_style_all.sh" ]; then \
		echo "[INFO] Formatting with astyle"; \
		"$(TOOLS_DIR)/astyle/check_code_style_all.sh" --fix; \
	else \
		echo "[WARN] Formatter script not found or not executable:"; \
		echo "       $(TOOLS_DIR)/astyle/check_code_style_all.sh"; \
	fi

# -------------------------
# Build targets
# -------------------------

debug: check-env
	@echo "[INFO] colcon build (Debug, -O0, symbols)"
	@$(call WITH_ENV, colcon build --symlink-install \
		--cmake-args \
			-DCMAKE_BUILD_TYPE=Debug \
			-DCMAKE_CXX_FLAGS="-O0 -g3 -ggdb3 -fno-omit-frame-pointer -Wall -Wextra -Wpedantic" \
			-DCMAKE_C_FLAGS="-O0 -g3 -ggdb3 -fno-omit-frame-pointer -Wall -Wextra -Wpedantic")

release: check-env
	@echo "[INFO] colcon build (Release)"
	@$(call WITH_ENV, colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)

clean:
	@echo "[INFO] clean build/install/log"
	@rm -rf build install log

debug-clean: clean debug
release-clean: clean release

# -------------------------
# Verification / inspection
# -------------------------

list-bin: check-env
	@$(call WITH_ENV, P=$$(ros2 pkg prefix sunhawk_debug); echo "[INFO] prefix=$$P"; ls -lah "$$P/lib/sunhawk_debug")