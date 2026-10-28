/*
 * cluster_telemetry.cpp - ETS2/ATS telemetry plugin for the BMW E9x cluster rig.
 *
 * The game loads this as bin/linux_x64/plugins/cluster-telemetry.so, and it
 * publishes a 512 byte snapshot of the truck into /dev/shm/ClusterTelemetry
 * once per simulation frame. host/cluster_sim/ets2_plugin.py picks it up and
 * the existing mapping turns it into CAN frames.
 *
 * Why a plugin of our own rather than RenCloud's: that project's layout is a
 * 32 kB struct covering trailers, jobs and gameplay events, published by a
 * Windows-first codebase whose Linux support lives in a fork. The cluster
 * needs about thirty numbers. A small block we define ourselves is one file
 * to keep in step with the Python instead of a whole struct to mirror, and it
 * builds here with nothing but g++.
 *
 * Threading: every SDK callback arrives on the game's simulation thread, so
 * the staging copy needs no locking. The only cross-process concern is a
 * reader catching a half-written frame, which the seqlock in publish() closes.
 */

#include <cerrno>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "scssdk_telemetry.h"
#include "common/scssdk_telemetry_common_channels.h"
#include "common/scssdk_telemetry_common_configs.h"
#include "common/scssdk_telemetry_truck_common_channels.h"
#include "eurotrucks2/scssdk_eut2.h"
#include "eurotrucks2/scssdk_telemetry_eut2.h"
#include "amtrucks/scssdk_ats.h"
#include "amtrucks/scssdk_telemetry_ats.h"

#include "cluster_shm.h"

/* Bumped by hand when the published fields change meaning. The Python side
   reports it so a stale .so left in the plugins folder is obvious. */
#define CLUSTER_PLUGIN_BUILD 1u

namespace {

/* Where the callbacks accumulate the frame. Published as one block at
   frame end so a reader never sees a mix of two frames. */
cluster_shm_t g_state;

cluster_shm_t *g_shm = nullptr;
scs_log_t g_log = nullptr;

/* Increments twice per frame: odd while a frame is being copied out. */
uint32_t g_seq = 0;

void log_line(const scs_log_type_t type, const char *const fmt, ...)
    __attribute__((format(printf, 2, 3)));

void log_line(const scs_log_type_t type, const char *const fmt, ...)
{
    if (!g_log) {
        return;
    }
    char text[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    g_log(type, text);
}

/* ---------------------------------------------------------------------- */
/* Channel callbacks                                                       */
/* ---------------------------------------------------------------------- */

/*
 * Each channel is registered with its destination as the context pointer, so
 * these four functions cover every channel we want. All of them are
 * registered with FLAG_no_value, which means the game calls us with a null
 * value when a channel has nothing to report - a truck that has just been
 * despawned, or a load screen. Zeroing on null is what keeps the gauges from
 * freezing at the last value they saw before the menu.
 */

SCSAPI_VOID store_float(const scs_string_t /*name*/, const scs_u32_t /*index*/,
                        const scs_value_t *const value, const scs_context_t context)
{
    float *const dst = static_cast<float *>(context);
    *dst = (value && value->type == SCS_VALUE_TYPE_float) ? value->value_float.value : 0.0f;
}

SCSAPI_VOID store_bool(const scs_string_t /*name*/, const scs_u32_t /*index*/,
                       const scs_value_t *const value, const scs_context_t context)
{
    uint8_t *const dst = static_cast<uint8_t *>(context);
    *dst = (value && value->type == SCS_VALUE_TYPE_bool && value->value_bool.value) ? 1u : 0u;
}

SCSAPI_VOID store_s32(const scs_string_t /*name*/, const scs_u32_t /*index*/,
                      const scs_value_t *const value, const scs_context_t context)
{
    int32_t *const dst = static_cast<int32_t *>(context);
    *dst = (value && value->type == SCS_VALUE_TYPE_s32) ? value->value_s32.value : 0;
}

SCSAPI_VOID store_u32(const scs_string_t /*name*/, const scs_u32_t /*index*/,
                      const scs_value_t *const value, const scs_context_t context)
{
    uint32_t *const dst = static_cast<uint32_t *>(context);
    *dst = (value && value->type == SCS_VALUE_TYPE_u32) ? value->value_u32.value : 0u;
}

/* For a channel the game publishes as u32 that we keep as a signed field,
   because every other gear-ish number here is signed. truck.hshifter.slot is
   the one that does this; registering it as s32 is refused outright with
   "type 2 is not supported by channel". */
SCSAPI_VOID store_u32_as_s32(const scs_string_t /*name*/, const scs_u32_t /*index*/,
                             const scs_value_t *const value, const scs_context_t context)
{
    int32_t *const dst = static_cast<int32_t *>(context);
    *dst = (value && value->type == SCS_VALUE_TYPE_u32)
               ? static_cast<int32_t>(value->value_u32.value)
               : 0;
}

/* ---------------------------------------------------------------------- */
/* Publishing                                                              */
/* ---------------------------------------------------------------------- */

void publish()
{
    if (!g_shm) {
        return;
    }

    g_state.frames++;

    /* Fields ahead of seq - magic, version, size - are written once at init
       and must survive, so the copy starts after seq rather than clobbering
       the lock we are holding. */
    const size_t head = offsetof(cluster_shm_t, seq) + sizeof(uint32_t);

    __atomic_store_n(&g_shm->seq, ++g_seq, __ATOMIC_RELEASE); /* odd: writing */
    memcpy(reinterpret_cast<char *>(g_shm) + head,
           reinterpret_cast<const char *>(&g_state) + head,
           sizeof(cluster_shm_t) - head);
    __atomic_store_n(&g_shm->seq, ++g_seq, __ATOMIC_RELEASE); /* even: settled */
}

/* ---------------------------------------------------------------------- */
/* Events                                                                  */
/* ---------------------------------------------------------------------- */

SCSAPI_VOID event_frame_start(const scs_event_t /*event*/, const void *const event_info,
                              const scs_context_t /*context*/)
{
    const scs_telemetry_frame_start_t *const info =
        static_cast<const scs_telemetry_frame_start_t *>(event_info);
    g_state.timestamp_us = info->simulation_time;
}

SCSAPI_VOID event_frame_end(const scs_event_t /*event*/, const void *const /*event_info*/,
                            const scs_context_t /*context*/)
{
    publish();
}

SCSAPI_VOID event_paused(const scs_event_t event, const void *const /*event_info*/,
                         const scs_context_t /*context*/)
{
    g_state.paused = (event == SCS_TELEMETRY_EVENT_paused) ? 1u : 0u;
    publish(); /* so a reader sees the pause immediately, not at the next frame */
}

void copy_string(char *const dst, const size_t cap, const scs_value_t *const value)
{
    if (!value || value->type != SCS_VALUE_TYPE_string || !value->value_string.value) {
        dst[0] = '\0';
        return;
    }
    snprintf(dst, cap, "%s", value->value_string.value);
}

SCSAPI_VOID event_configuration(const scs_event_t /*event*/, const void *const event_info,
                                const scs_context_t /*context*/)
{
    const scs_telemetry_configuration_t *const info =
        static_cast<const scs_telemetry_configuration_t *>(event_info);

    /* Trailer, job and hshifter configs arrive here too; the cluster only
       cares what the truck under it is. */
    if (strcmp(info->id, SCS_TELEMETRY_CONFIG_truck) != 0) {
        return;
    }

    /* A truck change re-sends the whole set, but only for the wheels the new
       truck has. Clearing first stops a 6-wheeler inheriting axles 7 and 8
       from the one before it, which would read as permanent wheelspin. */
    memset(g_state.wheel_radius, 0, sizeof(g_state.wheel_radius));
    memset(g_state.wheel_powered, 0, sizeof(g_state.wheel_powered));
    memset(g_state.wheel_steerable, 0, sizeof(g_state.wheel_steerable));
    memset(g_state.wheel_simulated, 0, sizeof(g_state.wheel_simulated));

    for (const scs_named_value_t *attr = info->attributes; attr && attr->name; ++attr) {
        const scs_value_t *const v = &attr->value;
        const scs_u32_t i = attr->index;
        const bool indexed = (i != SCS_U32_NIL) && (i < CLUSTER_WHEELS);

        if (strcmp(attr->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_fuel_capacity) == 0) {
            g_state.fuel_capacity = v->value_float.value;
        } else if (strcmp(attr->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_rpm_limit) == 0) {
            g_state.engine_rpm_max = v->value_float.value;
        } else if (strcmp(attr->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_forward_gear_count) == 0) {
            g_state.gears_forward = static_cast<int32_t>(v->value_u32.value);
        } else if (strcmp(attr->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_count) == 0) {
            const scs_u32_t n = v->value_u32.value;
            g_state.wheel_count = (n > CLUSTER_WHEELS) ? CLUSTER_WHEELS : n;
        } else if (indexed &&
                   strcmp(attr->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_radius) == 0) {
            g_state.wheel_radius[i] = v->value_float.value;
        } else if (indexed &&
                   strcmp(attr->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_powered) == 0) {
            g_state.wheel_powered[i] = v->value_bool.value ? 1u : 0u;
        } else if (indexed &&
                   strcmp(attr->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_steerable) == 0) {
            g_state.wheel_steerable[i] = v->value_bool.value ? 1u : 0u;
        } else if (indexed &&
                   strcmp(attr->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_simulated) == 0) {
            g_state.wheel_simulated[i] = v->value_bool.value ? 1u : 0u;
        } else if (strcmp(attr->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_brand) == 0) {
            copy_string(g_state.truck_brand, sizeof(g_state.truck_brand), v);
        } else if (strcmp(attr->name, SCS_TELEMETRY_CONFIG_ATTRIBUTE_name) == 0) {
            copy_string(g_state.truck_name, sizeof(g_state.truck_name), v);
        }
    }

    log_line(SCS_LOG_TYPE_message,
             "cluster-telemetry: truck '%s %s', %u wheels, %.0f l tank, %.0f rpm limit",
             g_state.truck_brand, g_state.truck_name, g_state.wheel_count,
             static_cast<double>(g_state.fuel_capacity),
             static_cast<double>(g_state.engine_rpm_max));

    publish();
}

/* ---------------------------------------------------------------------- */
/* Shared memory                                                           */
/* ---------------------------------------------------------------------- */

/*
 * Which block to publish. Normally the one name the host looks for, but
 * CLUSTER_SHM_NAME in the environment overrides it, which is what lets the
 * test harness run while a game is playing: the harness loads this very
 * library and its shutdown unlinks whatever it opened, so sharing a name
 * with a live game would delete that game's block out from under it.
 *
 * Only a plain "/name" is accepted - no embedded slashes, nothing that could
 * wander out of the shm namespace - and anything else falls back to the
 * default rather than failing, since a typo here should cost the override,
 * not the telemetry.
 */
const char *shm_name()
{
    const char *const env = getenv("CLUSTER_SHM_NAME");
    if (!env || env[0] != '/' || env[1] == '\0') {
        return CLUSTER_SHM_NAME;
    }
    for (const char *p = env + 1; *p; ++p) {
        if (*p == '/') {
            return CLUSTER_SHM_NAME;
        }
    }
    return env;
}

/* Remembered for shutdown: getenv could in principle change underneath us,
   and unlinking a different name than we created would leave the block
   behind for ever. */
const char *g_shm_name = CLUSTER_SHM_NAME;

bool open_shm()
{
    g_shm_name = shm_name();
    const int fd = shm_open(g_shm_name, O_CREAT | O_RDWR, 0600);
    if (fd < 0) {
        log_line(SCS_LOG_TYPE_error, "cluster-telemetry: shm_open(%s) failed: %s",
                 g_shm_name, strerror(errno));
        return false;
    }

    if (ftruncate(fd, CLUSTER_SHM_SIZE) != 0) {
        log_line(SCS_LOG_TYPE_error, "cluster-telemetry: ftruncate failed: %s", strerror(errno));
        close(fd);
        return false;
    }

    void *const base = mmap(nullptr, CLUSTER_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    /* The mapping outlives the descriptor, and holding the fd open gains us
       nothing, so it goes back now rather than leaking across a reinit. */
    close(fd);

    if (base == MAP_FAILED) {
        log_line(SCS_LOG_TYPE_error, "cluster-telemetry: mmap failed: %s", strerror(errno));
        return false;
    }

    g_shm = static_cast<cluster_shm_t *>(base);
    memset(g_shm, 0, CLUSTER_SHM_SIZE);
    return true;
}

void close_shm()
{
    if (!g_shm) {
        return;
    }
    /* Clear magic before unmapping: a reader that still has the block open
       should see "no plugin" rather than the last frame of a finished game. */
    g_shm->magic = 0;
    g_shm->sdk_active = 0;
    munmap(g_shm, CLUSTER_SHM_SIZE);
    g_shm = nullptr;

    /* Unlink so the next run starts from a fresh block. The name reappears
       the moment the game loads the plugin again. */
    shm_unlink(g_shm_name);
}

/* ---------------------------------------------------------------------- */
/* Registration                                                            */
/* ---------------------------------------------------------------------- */

/*
 * every_frame: the game calls us once per frame whatever the value did, and
 * with a null value when the channel has nothing. Change-driven registration
 * would be cheaper, but this way a reader can tell a frozen gauge from a
 * frozen game by watching the frame counter alone.
 */
const scs_u32_t kFlags =
    SCS_TELEMETRY_CHANNEL_FLAG_each_frame | SCS_TELEMETRY_CHANNEL_FLAG_no_value;

#define REG(params, chan, type, store, field) \
    (params)->register_for_channel(chan, SCS_U32_NIL, type, kFlags, store, &g_state.field)

#define REG_I(params, chan, idx, type, store, field) \
    (params)->register_for_channel(chan, idx, type, kFlags, store, &g_state.field)

void register_channels(const scs_telemetry_init_params_v100_t *const p)
{
#define F(chan, field) REG(p, chan, SCS_VALUE_TYPE_float, store_float, field)
#define B(chan, field) REG(p, chan, SCS_VALUE_TYPE_bool, store_bool, field)
#define I(chan, field) REG(p, chan, SCS_VALUE_TYPE_s32, store_s32, field)
#define U(chan, field) REG(p, chan, SCS_VALUE_TYPE_u32, store_u32, field)

    U(SCS_TELEMETRY_CHANNEL_game_time, time_abs);

    F(SCS_TELEMETRY_TRUCK_CHANNEL_speed, speed);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_engine_rpm, engine_rpm);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_fuel, fuel);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_fuel_range, fuel_range);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_fuel_average_consumption, fuel_avg_consumption);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_adblue, adblue);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_water_temperature, water_temperature);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_oil_pressure, oil_pressure);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_oil_temperature, oil_temperature);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_brake_air_pressure, brake_air_pressure);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_battery_voltage, battery_voltage);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_effective_throttle, throttle);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_effective_brake, brake);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_effective_clutch, clutch);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_effective_steering, steering);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_odometer, odometer);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_cruise_control, cruise_control);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_dashboard_backlight, dashboard_backlight);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_wear_engine, wear_engine);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_wear_transmission, wear_transmission);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_navigation_speed_limit, navigation_speed_limit);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_navigation_distance, navigation_distance);
    F(SCS_TELEMETRY_TRUCK_CHANNEL_navigation_time, navigation_time);

    I(SCS_TELEMETRY_TRUCK_CHANNEL_engine_gear, gear);
    I(SCS_TELEMETRY_TRUCK_CHANNEL_displayed_gear, displayed_gear);
    REG(p, SCS_TELEMETRY_TRUCK_CHANNEL_hshifter_slot, SCS_VALUE_TYPE_u32,
        store_u32_as_s32, hshifter_slot);

    B(SCS_TELEMETRY_TRUCK_CHANNEL_electric_enabled, electric_enabled);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_engine_enabled, engine_enabled);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_parking_brake, park_brake);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_motor_brake, motor_brake);

    B(SCS_TELEMETRY_TRUCK_CHANNEL_light_parking, light_parking);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_light_low_beam, light_low_beam);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_light_high_beam, light_high_beam);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_light_beacon, light_beacon);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_light_brake, light_brake);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_light_reverse, light_reverse);

    B(SCS_TELEMETRY_TRUCK_CHANNEL_lblinker, blinker_left_active);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_rblinker, blinker_right_active);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_light_lblinker, blinker_left_on);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_light_rblinker, blinker_right_on);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_hazard_warning, hazard);

    B(SCS_TELEMETRY_TRUCK_CHANNEL_differential_lock, differential_lock);

    B(SCS_TELEMETRY_TRUCK_CHANNEL_fuel_warning, fuel_warning);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_adblue_warning, adblue_warning);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_oil_pressure_warning, oil_pressure_warning);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_water_temperature_warning, water_temperature_warning);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_battery_voltage_warning, battery_voltage_warning);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_brake_air_pressure_warning, air_pressure_warning);
    B(SCS_TELEMETRY_TRUCK_CHANNEL_brake_air_pressure_emergency, air_pressure_emergency);

#undef F
#undef B
#undef I
#undef U

    /* Wheels are indexed channels: one registration per axle end.
     *
     * The game caps the index at however many wheels it supports - 14 on
     * ETS2 1.61 - and refuses anything above it with "index N is not valid
     * for channel". Stopping at the first refusal keeps the array at 16 for
     * the host's benefit while asking only for what this build has, and
     * adapts by itself if a later patch moves the cap. It costs one error
     * line in game.log.txt at startup, which is the game telling us where
     * the edge is. */
    for (scs_u32_t i = 0; i < CLUSTER_WHEELS; ++i) {
        if (REG_I(p, SCS_TELEMETRY_TRUCK_CHANNEL_wheel_velocity, i, SCS_VALUE_TYPE_float,
                  store_float, wheel_angular_velocity[i]) != SCS_RESULT_ok) {
            log_line(SCS_LOG_TYPE_message, "cluster-telemetry: %u wheel channels", i);
            break;
        }
        REG_I(p, SCS_TELEMETRY_TRUCK_CHANNEL_wheel_on_ground, i, SCS_VALUE_TYPE_bool,
              store_bool, wheel_on_ground[i]);
    }
}

} // namespace

/* ---------------------------------------------------------------------- */
/* SDK entry points                                                        */
/* ---------------------------------------------------------------------- */

/*
 * These get C linkage from scssdk_telemetry.h, which declares them inside
 * SCSSDK_HEADER - that macro is extern "C" {. The game resolves them by name
 * with dlsym, so they must not be mangled and must not be hidden.
 *
 * The explicit visibility is not redundant with plugin.map: the whole file is
 * built -fvisibility=hidden, and a version script can only choose among
 * symbols that reached the linker as dynamic. Without this the library builds,
 * loads, exports nothing, and the game walks past it in silence.
 */
#define CLUSTER_EXPORT __attribute__((visibility("default")))

CLUSTER_EXPORT SCSAPI_RESULT scs_telemetry_init(const scs_u32_t version,
                                                const scs_telemetry_init_params_t *const params)
{
    /* 1.00 and 1.01 are layout-identical; anything else is a version of the
       API this was not written against, and guessing would be worse than
       declining. The game logs the refusal. */
    if (version != SCS_TELEMETRY_VERSION_1_00 && version != SCS_TELEMETRY_VERSION_1_01) {
        return SCS_RESULT_unsupported;
    }

    const scs_telemetry_init_params_v100_t *const p =
        static_cast<const scs_telemetry_init_params_v100_t *>(params);

    g_log = p->common.log;

    memset(&g_state, 0, sizeof(g_state));

    if (!open_shm()) {
        g_log = nullptr;
        return SCS_RESULT_generic_error;
    }

    g_state.magic = CLUSTER_SHM_MAGIC;
    g_state.version = CLUSTER_SHM_VERSION;
    g_state.size = static_cast<uint32_t>(sizeof(cluster_shm_t));
    g_state.plugin_build = CLUSTER_PLUGIN_BUILD;
    g_state.sdk_active = 1;
    g_state.paused = 1; /* until the first frame says otherwise */
    g_state.game_major = SCS_GET_MAJOR_VERSION(p->common.game_version);
    g_state.game_minor = SCS_GET_MINOR_VERSION(p->common.game_version);

    snprintf(g_state.game_id, sizeof(g_state.game_id), "%s", p->common.game_id);
    if (strcmp(p->common.game_id, SCS_GAME_ID_EUT2) == 0) {
        g_state.game = CLUSTER_GAME_ETS2;
    } else if (strcmp(p->common.game_id, SCS_GAME_ID_ATS) == 0) {
        g_state.game = CLUSTER_GAME_ATS;
    } else {
        g_state.game = CLUSTER_GAME_UNKNOWN;
    }

    /* The header is published before any frame arrives so a reader starting
       first can identify the block instead of waiting for the game to be
       unpaused. */
    g_shm->magic = g_state.magic;
    g_shm->version = g_state.version;
    g_shm->size = g_state.size;
    publish();

    const bool events_ok =
        p->register_for_event(SCS_TELEMETRY_EVENT_frame_start, event_frame_start, nullptr) ==
            SCS_RESULT_ok &&
        p->register_for_event(SCS_TELEMETRY_EVENT_frame_end, event_frame_end, nullptr) ==
            SCS_RESULT_ok &&
        p->register_for_event(SCS_TELEMETRY_EVENT_paused, event_paused, nullptr) ==
            SCS_RESULT_ok &&
        p->register_for_event(SCS_TELEMETRY_EVENT_started, event_paused, nullptr) ==
            SCS_RESULT_ok &&
        p->register_for_event(SCS_TELEMETRY_EVENT_configuration, event_configuration, nullptr) ==
            SCS_RESULT_ok;

    if (!events_ok) {
        log_line(SCS_LOG_TYPE_error, "cluster-telemetry: could not register events");
        close_shm();
        g_log = nullptr;
        return SCS_RESULT_generic_error;
    }

    /* Channel registration failures are not fatal on purpose: a channel the
       running game does not have should cost us that one gauge, not the
       whole plugin. The ones that matter are old and stable. */
    register_channels(p);

    log_line(SCS_LOG_TYPE_message,
             "cluster-telemetry: build %u up on %s telemetry %u.%u, %u bytes at %s",
             CLUSTER_PLUGIN_BUILD, p->common.game_id, g_state.game_major, g_state.game_minor,
             g_state.size, g_shm_name);

    return SCS_RESULT_ok;
}

CLUSTER_EXPORT SCSAPI_VOID scs_telemetry_shutdown(void)
{
    close_shm();
    g_log = nullptr;
}

/*
 * The game may dlclose() us without calling shutdown in some failure paths,
 * so the mapping is also released from a destructor. Both are idempotent.
 */
__attribute__((destructor)) static void on_unload(void)
{
    close_shm();
}
