SDK   ?= /home/rusty/Projects/embed/pico-sdk/pico-sdk
BUILD := build
PORT  ?= /dev/ttyACM0

.PHONY: all configure build flash verify info watch clean lsp

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

watch:
	@until [ -e $(PORT) ]; do sleep 0.1; done
	minicom -b 115200 -D $(PORT)

clean:
	rm -rf $(BUILD)

$(BUILD)/Makefile:
	$(MAKE) configure
