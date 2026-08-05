BUILD_DIR_PICO = build/pico
BUILD_DIR_PICO2 = build/pico2
TEST_DIR = tests
TEST_BUILD_DIR = build/tests
TEST_SRC = $(TEST_DIR)/test_main.c $(TEST_SUITE_SRC)
TEST_SUITE_SRC = $(filter-out $(TEST_DIR)/test_main.c,$(wildcard $(TEST_DIR)/test_*.c))
TEST_STUB_SRC = $(wildcard $(TEST_DIR)/stubs/*.c)
TEST_UNITY_SRC = $(TEST_DIR)/unity/unity.c
TEST_APP_SRC = src/usb_comm.c src/runtime_config.c src/esc_firmware/update.c src/pwm/control.c src/pwm/pwm.c src/dshot/control.c src/dshot/telemetry_usb.c
CMAKE_FLAGS = -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DPICO_SDK_FETCH_FROM_GIT=ON -DPython3_EXECUTABLE=$(shell which python3)
CMAKE_FLAGS_PICO2 = $(CMAKE_FLAGS) -DPICO_BOARD=pico2
ARM_GCC_INCLUDE = $(shell arm-none-eabi-gcc -print-file-name=include)
SYSROOT_A = $(shell arm-none-eabi-gcc -print-sysroot)/include
SYSROOT_B = /usr/arm-none-eabi/include
SYSROOT_C = /usr/lib/arm-none-eabi/include

.PHONY: build-pico build-pico2 flash-pico flash-pico2 clean format format-check lint lint-check test help

build-pico:
	mkdir -p $(BUILD_DIR_PICO)
	cd $(BUILD_DIR_PICO) && cmake -S $(CURDIR) -B . $(CMAKE_FLAGS) && cmake --build .

build-pico2:
	mkdir -p $(BUILD_DIR_PICO2)
	cd $(BUILD_DIR_PICO2) && cmake -S $(CURDIR) -B . $(CMAKE_FLAGS_PICO2) && cmake --build .

flash-pico: build-pico
	picotool load $(BUILD_DIR_PICO)/firmware.uf2 -f

flash-pico2: build-pico2
	picotool load $(BUILD_DIR_PICO2)/firmware.uf2 -f

clean:
	rm -rf build

format:
	find . \( -name "*.c" -o -name "*.h" \) ! -path "./build/*" ! -path "./tests/unity/*" | xargs clang-format -i

format-check:
	find . \( -name "*.c" -o -name "*.h" \) ! -path "./build/*" ! -path "./tests/unity/*" | xargs clang-format --dry-run --Werror

lint: build-pico
	find src -name "*.c" | xargs clang-tidy --fix-errors \
	-p $(BUILD_DIR_PICO)/compile_commands.json \
	-header-filter="^$(CURDIR)/src/.*" \
	--extra-arg=-I$(ARM_GCC_INCLUDE) \
	--extra-arg=-I$(SYSROOT_A) \
	--extra-arg=-I$(SYSROOT_B) \
	--extra-arg=-I$(SYSROOT_C)

lint-check: build-pico
	find src -name "*.c" | xargs clang-tidy --warnings-as-errors=* \
	-p $(BUILD_DIR_PICO)/compile_commands.json \
	-header-filter="^$(CURDIR)/src/.*" \
	--extra-arg=-I$(ARM_GCC_INCLUDE) \
	--extra-arg=-I$(SYSROOT_A) \
	--extra-arg=-I$(SYSROOT_B) \
	--extra-arg=-I$(SYSROOT_C)

test:
	mkdir -p $(TEST_BUILD_DIR)
	cc -std=c11 -Wall -Wextra -I$(TEST_DIR)/mocks -I$(TEST_DIR) -Isrc \
		$(TEST_SRC) $(TEST_STUB_SRC) $(TEST_UNITY_SRC) $(TEST_APP_SRC) \
		-o $(TEST_BUILD_DIR)/run_tests
	./$(TEST_BUILD_DIR)/run_tests

help:
	@echo "Available targets:"
	@echo "  build-pico      - Build firmware for Pico"
	@echo "  build-pico2     - Build firmware for Pico 2"
	@echo "  flash-pico         - Build and flash unified firmware for Pico"
	@echo "  flash-pico2        - Build and flash unified firmware for Pico 2"
	@echo "  clean           - Clean build dirs"
	@echo "  format          - Format C code"
	@echo "  format-check    - Check C code formatting"
	@echo "  lint            - Lint C code (auto-fix errors)"
	@echo "  lint-check      - Check C code linting (report only)"
	@echo "  test            - Build and run host unit tests"
	@echo "  help            - Show this help"
