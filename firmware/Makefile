SDK   ?= lib/pico-sdk
BUILD := build
PORT  ?= /dev/ttyACM0

.PHONY: all configure build flash verify info watch clean lsp \
        motor_test motor_test_flash \
        engine_ramp engine_ramp_flash watch_engine_ramp

all: build

configure:
	cmake -B $(BUILD) -DPICO_SDK_PATH=$(SDK) -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

lsp: $(BUILD)/Makefile
	ln -sf $(BUILD)/compile_commands.json compile_commands.json

build: $(BUILD)/Makefile
	cmake --build $(BUILD) -j$(nproc)

flash: build
	cmake --build $(BUILD) --target flash

verify: build
	cmake --build $(BUILD) --target verify

info:
	cmake --build $(BUILD) --target info

motor_test: $(BUILD)/Makefile
	cmake --build $(BUILD) --target motor_test -j$(nproc)

motor_test_flash: motor_test
	cmake --build $(BUILD) --target motor_test_flash

engine_ramp: $(BUILD)/Makefile
	cmake --build $(BUILD) --target engine_ramp -j$(nproc)

engine_ramp_flash: engine_ramp
	cmake --build $(BUILD) --target engine_ramp_flash

watch:
	@until [ -e $(PORT) ]; do sleep 0.1; done
	minicom -b 115200 -D $(PORT)

watch_engine_ramp:
	@until [ -e $(PORT) ]; do sleep 0.1; done
	cat $(PORT)

clean:
	rm -rf $(BUILD)

$(BUILD)/Makefile:
	$(MAKE) configure
