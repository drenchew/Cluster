#!/usr/bin/env bash
#
# Get the cluster ready to drive from ETS2/ATS, from however broken a state.
#
#   ./drive.sh              fix what needs fixing, then run the bridge
#   ./drive.sh --check      say what is wrong, change nothing
#   ./drive.sh --launch     also start the game from Steam first
#   ./drive.sh --ats        American Truck Simulator instead of ETS2
#
# Anything after -- goes to run_ets2.py:
#
#   ./drive.sh -- --rpm-scale 1.5 --tank-l 70
#
# The two things that break between sessions, and what this does about them:
#
#   A game update can wipe bin/linux_x64/plugins/, because that lives inside
#   the game's own install. So the plugin is rebuilt and reinstalled whenever
#   the installed copy is missing or does not match what we build.
#
#   Unplugging the Arduino brings /dev/ttyUSB0 back as root:dialout 660. If
#   you are not in dialout that is a permission error, so this offers the
#   one-time group fix plus a chmod that works right now. It needs sudo, and
#   will say so before asking.

set -uo pipefail

# Kept for the dialout re-exec below, which has to restart with what it was
# originally given.
ORIG_ARGS=("$@")

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLUGIN_DIR="$REPO/plugin"
HOST_DIR="$REPO/host"
SO_NAME="cluster-telemetry.so"

GAME="ets2"
CHECK_ONLY=0
LAUNCH=0
PASSTHRU=()

while [ $# -gt 0 ]; do
    case "$1" in
        --check)  CHECK_ONLY=1 ;;
        --launch) LAUNCH=1 ;;
        --ats)    GAME="ats" ;;
        --ets2)   GAME="ets2" ;;
        -h|--help) sed -n '2,26p' "${BASH_SOURCE[0]}" | sed 's/^#\?//'; exit 0 ;;
        --)       shift; PASSTHRU=("$@"); break ;;
        *)        echo "unknown option: $1  (try --help)" >&2; exit 2 ;;
    esac
    shift
done

if [ "$GAME" = "ats" ]; then
    GAME_NAME="American Truck Simulator"
    GAME_BIN="amtrucks"
    APPID=270880
else
    GAME_NAME="Euro Truck Simulator 2"
    GAME_BIN="eurotrucks2"
    APPID=227300
fi

red()   { printf '\033[31m%s\033[0m\n' "$*"; }
green() { printf '\033[32m%s\033[0m\n' "$*"; }
warn()  { printf '\033[33m%s\033[0m\n' "$*"; }
step()  { printf '\n\033[1m== %s\033[0m\n' "$*"; }

PROBLEMS=0
note_problem() { PROBLEMS=$((PROBLEMS + 1)); }

# ---------------------------------------------------------------------------
# 1. Find the game
# ---------------------------------------------------------------------------
# Steam spreads installs across libraries listed in libraryfolders.vdf, and
# the Flatpak build keeps its own tree, so guessing one path is not enough.

step "Finding $GAME_NAME"

find_game() {
    if [ -n "${GAME_DIR:-}" ]; then
        echo "$GAME_DIR"
        return
    fi
    local roots=(
        "$HOME/.local/share/Steam"
        "$HOME/.steam/steam"
        "$HOME/.var/app/com.valvesoftware.Steam/.local/share/Steam"
        "$HOME/snap/steam/common/.local/share/Steam"
    )
    # Extra libraries on other drives.
    local vdf path
    for vdf in "${roots[@]/%//steamapps/libraryfolders.vdf}"; do
        [ -r "$vdf" ] || continue
        while read -r path; do
            [ -n "$path" ] && roots+=("$path")
        done < <(grep -oP '"path"\s*"\K[^"]+' "$vdf" 2>/dev/null)
    done
    local root dir
    for root in "${roots[@]}"; do
        dir="$root/steamapps/common/$GAME_NAME"
        if [ -x "$dir/bin/linux_x64/$GAME_BIN" ]; then
            echo "$dir"
            return
        fi
    done
}

GAME_DIR="$(find_game)"

if [ -z "$GAME_DIR" ]; then
    red "could not find a native Linux $GAME_NAME"
    echo "  Looked through every Steam library for"
    echo "    steamapps/common/$GAME_NAME/bin/linux_x64/$GAME_BIN"
    echo
    echo "  If the game is installed but that binary is missing, Steam has"
    echo "  forced Proton on it: Properties -> Compatibility -> untick"
    echo "  \"Force the use of a specific Steam Play compatibility tool\"."
    echo "  Telemetry cannot work under Proton - the shared memory stays"
    echo "  inside the Wine prefix."
    echo
    echo "  Or point this at it:  GAME_DIR=/path/to/game $0"
    exit 1
fi
green "$GAME_DIR"

INSTALLED="$GAME_DIR/bin/linux_x64/plugins/$SO_NAME"
BUILT="$PLUGIN_DIR/plugins/$SO_NAME"

# ---------------------------------------------------------------------------
# 2. Build and install the plugin if it is missing or stale
# ---------------------------------------------------------------------------

step "Checking the telemetry plugin"

need_build=0
if [ ! -f "$BUILT" ]; then
    echo "not built yet"
    need_build=1
else
    # Rebuild if any source is newer than the binary.
    while read -r src; do
        if [ "$src" -nt "$BUILT" ]; then
            echo "$(basename "$src") is newer than the build"
            need_build=1
            break
        fi
    done < <(find "$PLUGIN_DIR/src" "$PLUGIN_DIR/include" -type f \
                  \( -name '*.cpp' -o -name '*.h' \) 2>/dev/null)
fi

if [ "$need_build" = 1 ]; then
    if [ "$CHECK_ONLY" = 1 ]; then
        warn "would rebuild the plugin"
        note_problem
    elif ! command -v g++ >/dev/null; then
        red "g++ is not installed, so the plugin cannot be built"
        echo "  sudo apt install g++"
        exit 1
    else
        echo "building..."
        if ! make -s -C "$PLUGIN_DIR" >/dev/null; then
            red "the plugin failed to build"
            make -C "$PLUGIN_DIR"
            exit 1
        fi
        green "built"
    fi
fi

need_install=0
if [ ! -f "$INSTALLED" ]; then
    # The usual cause is a game update wiping the plugins folder.
    warn "the plugin is not installed in the game"
    need_install=1
elif [ -f "$BUILT" ] && ! cmp -s "$BUILT" "$INSTALLED"; then
    warn "the installed plugin differs from the one we build"
    need_install=1
fi

if [ "$need_install" = 1 ]; then
    if [ "$CHECK_ONLY" = 1 ]; then
        warn "would reinstall it"
        note_problem
    else
        if ! make -s -C "$PLUGIN_DIR" install GAME_DIR="$GAME_DIR" >/dev/null; then
            red "could not install the plugin"
            make -C "$PLUGIN_DIR" install GAME_DIR="$GAME_DIR"
            exit 1
        fi
        green "installed to $INSTALLED"
        if pgrep -x "$GAME_BIN" >/dev/null 2>&1; then
            warn "the game is running the old build - restart it, or type"
            warn "'sdk reload' in its console, to pick this up"
        fi
    fi
elif [ -f "$INSTALLED" ]; then
    green "installed and up to date"
fi

# ---------------------------------------------------------------------------
# 3. Serial port
# ---------------------------------------------------------------------------
# The jig is an Arduino on a USB-serial adapter. Replugging it recreates the
# device node as root:dialout 660, so this is the step that breaks most often.

step "Checking the serial port"

find_port() {
    if [ -n "${CLUSTER_PORT:-}" ]; then
        echo "$CLUSTER_PORT"
        return
    fi
    local p
    for p in /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$p" ] && { echo "$p"; return; }
    done
}

PORT="$(find_port)"

if [ -z "$PORT" ]; then
    warn "no serial adapter found (no /dev/ttyUSB* or /dev/ttyACM*)"
    echo "  Plug the jig in. The bridge will start anyway and connect when it"
    echo "  appears, so this is not fatal."
    note_problem
else
    echo "$PORT"
    if [ -w "$PORT" ]; then
        green "writable"
    else
        warn "not writable by you - this is what stops the cluster moving"
        note_problem

        # The common case after the group fix: you are a dialout member in
        # /etc/group, but this login session was created before that and
        # Linux stamps a session's groups at login. No sudo is needed - sg
        # starts a shell that does carry the group. The marker stops this
        # looping if the port is unreadable for some other reason.
        in_group_file=0
        getent group dialout 2>/dev/null | grep -qE "[:,]$USER(,|$)" && in_group_file=1
        in_session=0
        id -nG 2>/dev/null | tr ' ' '\n' | grep -qx dialout && in_session=1

        if [ "$in_group_file" = 1 ] && [ "$in_session" = 0 ]; then
            warn "you ARE in the dialout group, but this login session predates it"
            if [ "$CHECK_ONLY" = 1 ]; then
                echo "  would re-run itself under 'sg dialout' - no sudo needed."
                echo "  Log out and back in to make it permanent."
            elif [ -z "${CLUSTER_SG_RETRY:-}" ]; then
                echo "  re-running under a dialout session - no sudo needed"
                echo "  (log out and back in and this step disappears)"
                export CLUSTER_SG_RETRY=1
                exec sg dialout -c "$(printf '%q ' "$0" "${ORIG_ARGS[@]}")"
            else
                red "  still not writable even with dialout - something else is wrong"
            fi
        fi

        if [ "$CHECK_ONLY" = 1 ]; then
            echo "  otherwise: chmod now, and add you to dialout"
        else
            echo
            echo "  Two fixes, and sudo will ask for your password:"
            echo "    chmod 666 $PORT        - works now, lost on replug"
            echo "    usermod -aG dialout    - permanent, needs a new login"
            echo
            if sudo -n true 2>/dev/null || [ -t 0 ]; then
                sudo chmod 666 "$PORT" \
                    && green "  port opened up" \
                    || red "  chmod failed"
                if id -nG | tr ' ' '\n' | grep -qx dialout; then
                    green "  already in dialout - survives replugs"
                else
                    if sudo usermod -aG dialout "$USER"; then
                        green "  added to dialout - log out and in to make it stick"
                    else
                        red "  usermod failed"
                    fi
                fi
            else
                red "  no terminal to ask for a password on - run these yourself:"
                echo "    sudo chmod 666 $PORT && sudo usermod -aG dialout $USER"
            fi
        fi
    fi
fi

# ---------------------------------------------------------------------------
# 4. Python side
# ---------------------------------------------------------------------------

step "Checking the host"

PY=python3
if [ -x "$HOST_DIR/.venv/bin/python" ]; then
    PY="$HOST_DIR/.venv/bin/python"
fi

if ! "$PY" -c "import serial" 2>/dev/null; then
    red "pyserial is missing for $PY"
    echo "  pip install pyserial"
    [ "$CHECK_ONLY" = 1 ] && note_problem || exit 1
else
    green "$PY has pyserial"
fi

if ! (cd "$HOST_DIR" && "$PY" run_ets2.py --selftest >/dev/null 2>&1); then
    red "the readers failed their selftest - the struct layouts disagree"
    (cd "$HOST_DIR" && "$PY" run_ets2.py --selftest)
    exit 1
fi
green "layout and mapping selftests pass"

# ---------------------------------------------------------------------------
# 5. Game running?
# ---------------------------------------------------------------------------

step "Checking the game"

if pgrep -x "$GAME_BIN" >/dev/null 2>&1; then
    green "$GAME_NAME is running"
    if [ -e /dev/shm/ClusterTelemetry ]; then
        green "and publishing telemetry"
    else
        warn "but not publishing yet - that is normal in the menus, and the"
        warn "plugin only republishes once a save is loaded"
    fi
elif [ "$LAUNCH" = 1 ] && [ "$CHECK_ONLY" = 0 ]; then
    echo "launching via Steam..."
    steam "steam://rungameid/$APPID" >/dev/null 2>&1 &
    green "asked Steam to start it - load a save when it comes up"
else
    warn "not running"
    [ "$LAUNCH" = 0 ] && echo "  start it from Steam, or use --launch"
fi

# ---------------------------------------------------------------------------
# 6. Go
# ---------------------------------------------------------------------------

if [ "$CHECK_ONLY" = 1 ]; then
    echo
    if [ "$PROBLEMS" = 0 ]; then
        green "nothing to fix - ./drive.sh would go straight to driving"
    else
        warn "$PROBLEMS thing(s) to sort out - ./drive.sh would fix what it can"
    fi
    exit 0
fi

step "Driving"
echo "The bridge waits for the game and retries the port, so you can load a"
echo "save, quit to the menu or replug the jig without restarting it."
echo "Ctrl-C to stop."
echo

cd "$HOST_DIR" || exit 1
exec "$PY" run_ets2.py -v ${PORT:+--port "$PORT"} "${PASSTHRU[@]}"
