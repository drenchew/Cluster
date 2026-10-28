# ETS2/ATS telemetry plugin for the cluster rig.
#
#   make              build plugins/cluster-telemetry.so
#   make test         build it and drive it without the game
#   make install      copy it into the game's plugins folder
#   make uninstall    remove it again
#   make show         print what is currently in shared memory
#
# GAME_DIR is found from the Steam library by default; override it for a
# non-Steam copy or for American Truck Simulator:
#
#   make install GAME_DIR="$HOME/.local/share/Steam/steamapps/common/American Truck Simulator"

CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++14 -Wall -Wextra -fPIC -fvisibility=hidden \
            -fno-exceptions -fno-rtti
INCLUDES := -Iinclude -Iscs_sdk/include

# The game dlopen()s this and resolves scs_telemetry_init by name, so those
# two entry points have to stay visible even though everything else is
# hidden. -fvisibility=hidden plus SCSAPI's own default visibility would be
# the tidy way; the SDK does not mark them, so the linker script does it.
LDFLAGS  := -shared -Wl,--version-script=plugin.map -Wl,--no-undefined \
            -static-libgcc -static-libstdc++
LDLIBS   := -lrt

OUT      := plugins/cluster-telemetry.so
SRC      := src/cluster_telemetry.cpp

STEAM_COMMON := $(HOME)/.local/share/Steam/steamapps/common
GAME_DIR     ?= $(STEAM_COMMON)/Euro Truck Simulator 2
PLUGIN_DIR   := $(GAME_DIR)/bin/linux_x64/plugins

.PHONY: all test install uninstall show clean

all: $(OUT)

$(OUT): $(SRC) include/cluster_shm.h plugin.map
	@mkdir -p plugins
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRC) $(LDFLAGS) $(LDLIBS) -o $@
	@echo
	@echo "built $@"
	@nm -D --defined-only $@ | grep scs_telemetry || \
	  (echo "!! scs_telemetry_* not exported - the game would ignore this" && false)

# Loads the .so exactly as the game does - dlopen, hand it a fake init
# params struct, pump a frame - and checks what landed in shared memory.
# No game, no hardware.
test: $(OUT) test/host_test.cpp
	$(CXX) -O1 -std=c++14 -Wall -Wextra $(INCLUDES) \
	  test/host_test.cpp -ldl -lrt -o plugins/host_test
	./plugins/host_test ./$(OUT)

# Copy to a temporary name and rename over the target. If the game is
# running it has the old .so mapped, and writing to that file in place is a
# good way to hand it a SIGBUS mid-frame; rename leaves the old inode alone
# until the game lets go of it.
install: $(OUT)
	@test -d "$(GAME_DIR)" || \
	  (echo "no game at '$(GAME_DIR)' - pass GAME_DIR=..." && false)
	mkdir -p "$(PLUGIN_DIR)"
	cp $(OUT) "$(PLUGIN_DIR)/.cluster-telemetry.so.new"
	mv -f "$(PLUGIN_DIR)/.cluster-telemetry.so.new" \
	      "$(PLUGIN_DIR)/cluster-telemetry.so"
	@echo "installed to $(PLUGIN_DIR)/cluster-telemetry.so"
	@pgrep -x eurotrucks2 >/dev/null 2>&1 && \
	  echo "the game is running the old build - restart it, or type 'sdk reload' in its console" || true

uninstall:
	rm -f "$(PLUGIN_DIR)/cluster-telemetry.so"
	@echo "removed from $(PLUGIN_DIR)"

show:
	@cd ../host && python3 -m cluster_sim.ets2_plugin --watch

clean:
	rm -rf plugins
