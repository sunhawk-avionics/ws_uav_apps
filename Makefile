# ============================================================
# ws_uav_apps Makefile — ROS 2 (Humble) + colcon
# - 编译前自动 source Tools/setup_env.sh
# - Debug 构建支持断点调试 (-O0, -g3)
# ============================================================

SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c

WS_DIR     := $(shell pwd)
TOOLS_DIR  := $(WS_DIR)/Tools
ENV_SCRIPT := $(TOOLS_DIR)/setup_env.sh

# Helper: 在子 shell 中 source 环境后执行命令
define WITH_ENV
set +u; source "$(ENV_SCRIPT)" >/dev/null; set -u; $(1)
endef

.PHONY: help env check-env format \
        debug release \
        clean debug-clean release-clean \
        list-bin

help:
	@echo "===== ws_uav_apps Makefile ====="
	@echo ""
	@echo "编译:"
	@echo "  make debug          - Debug 构建 (-O0 -g3)，支持断点"
	@echo "  make release        - Release 构建"
	@echo ""
	@echo "清理:"
	@echo "  make clean          - 删除 build/install/log"
	@echo "  make debug-clean    - clean + debug"
	@echo "  make release-clean  - clean + release"
	@echo ""
	@echo "辅助:"
	@echo "  make env            - 打印 ROS 2 环境层级"
	@echo "  make format         - astyle 代码格式化"
	@echo "  make list-bin       - 列出 sunhawk_debug 已安装的可执行文件"

# -------------------------
# 环境检查
# -------------------------

check-env:
	@if [ ! -f "$(ENV_SCRIPT)" ]; then \
		echo "[ERROR] Missing env script: $(ENV_SCRIPT)"; \
		echo "        Please create Tools/setup_env.sh"; \
		exit 1; \
	fi

env: check-env
	@$(call WITH_ENV, echo "ROS_DISTRO=$$ROS_DISTRO"; echo "$$AMENT_PREFIX_PATH" | tr ":" "\n" | nl -ba)

# -------------------------
# 代码风格
# -------------------------

format:
	@if [ -x "$(TOOLS_DIR)/astyle/check_code_style_all.sh" ]; then \
		echo "[INFO] Formatting with astyle"; \
		"$(TOOLS_DIR)/astyle/check_code_style_all.sh" --fix; \
	else \
		echo "[WARN] Formatter script not found or not executable:"; \
		echo "       $(TOOLS_DIR)/astyle/check_code_style_all.sh"; \
	fi

# -------------------------
# 编译
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

# -------------------------
# 清理
# -------------------------

clean:
	@echo "[INFO] clean build/install/log"
	@rm -rf build install log

debug-clean: clean debug
release-clean: clean release

# -------------------------
# 检查 / 诊断
# -------------------------

list-bin: check-env
	@$(call WITH_ENV, P=$$(ros2 pkg prefix sunhawk_debug); echo "[INFO] prefix=$$P"; ls -lah "$$P/lib/sunhawk_debug")
