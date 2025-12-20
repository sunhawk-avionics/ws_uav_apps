SRC_DIR := $(shell dirname "$(realpath $(lastword $(MAKEFILE_LIST)))")

format:
	$(call colorecho,'Formatting with astyle')
	@"$(SRC_DIR)"/Tools/astyle/check_code_style_all.sh --fix