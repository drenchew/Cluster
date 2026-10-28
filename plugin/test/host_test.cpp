/*
 * host_test - drive the plugin without the game.
 *
 * The game loads the .so, hands it a table of function pointers, then calls
 * back into it once per frame. This does the same thing from a normal
 * process: dlopen, fake init params, deliver a truck configuration, pump one
 * frame of channel values, then read /dev/shm and check what came out.
 *
 * It is built as C++ and includes the SDK headers so the init params struct
 * is laid out by the same compiler rules the plugin was built with. Writing
 * it in C would mean hand-mirroring a struct with an empty base class, which
 * is the one way to get a test that passes while the real thing crashes.
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <dlfcn.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "scssdk_telemetry.h"
#include "common/scssdk_telemetry_common_channels.h"
#include "common/scssdk_telemetry_common_configs.h"
#include "common/scssdk_telemetry_truck_common_channels.h"
#include "eurotrucks2/scssdk_eut2.h"
#include "eurotrucks2/scssdk_telemetry_eut2.h"

#include "cluster_shm.h"

/* Deliberately not CLUSTER_SHM_NAME - see the setenv in main(). */
#define kTestShmName "/ClusterTelemetryTest"

static int g_failures = 0;

#define CHECK(cond, ...)                                 \
    do {                                                 \
        if (!(cond)) {                                   \
            printf("  FAIL: ");                          \
            printf(__VA_ARGS__);                         \
            printf("\n");                                \
            ++g_failures;                                \
        }                                                \
    } while (0)

/* ---------------------------------------------------------------------- */
/* The fake game                                                           */
/* ---------------------------------------------------------------------- */

struct ChannelReg {
    const char *name;
    scs_u32_t index;
    scs_value_type_t type;
    scs_telemetry_channel_callback_t callback;
    scs_context_t context;
};

struct EventReg {
    scs_event_t event;
    scs_telemetry_event_callback_t callback;
    scs_context_t context;
};

static ChannelReg g_channels[512];
static size_t g_channel_count = 0;
static EventReg g_events[16];
static size_t g_event_count = 0;

static SCSAPI_VOID fake_log(const scs_log_type_t type, const scs_string_t message)
{
    static const char *kNames[] = {"message", "warning", "error"};
    const char *label = (type >= 0 && type <= 2) ? kNames[type] : "?";
    printf("  [game %s] %s\n", label, message);
}

static SCSAPI_RESULT fake_register_channel(const scs_string_t name, const scs_u32_t index,
                                           const scs_value_type_t type, const scs_u32_t flags,
                                           const scs_telemetry_channel_callback_t callback,
                                           const scs_context_t context)
{
    (void)flags;
    if (g_channel_count >= sizeof(g_channels) / sizeof(g_channels[0])) {
        return SCS_RESULT_generic_error;
    }
    ChannelReg &r = g_channels[g_channel_count++];
    r.name = name;
    r.index = index;
    r.type = type;
    r.callback = callback;
    r.context = context;
    return SCS_RESULT_ok;
}

static SCSAPI_RESULT fake_unregister_channel(const scs_string_t, const scs_u32_t,
                                             const scs_value_type_t)
{
    return SCS_RESULT_ok;
}

static SCSAPI_RESULT fake_register_event(const scs_event_t event,
                                         const scs_telemetry_event_callback_t callback,
                                         const scs_context_t context)
{
    if (g_event_count >= sizeof(g_events) / sizeof(g_events[0])) {
        return SCS_RESULT_generic_error;
    }
    EventReg &r = g_events[g_event_count++];
    r.event = event;
    r.callback = callback;
    r.context = context;
    return SCS_RESULT_ok;
}

static SCSAPI_RESULT fake_unregister_event(const scs_event_t)
{
    return SCS_RESULT_ok;
}

static void fire_event(const scs_event_t event, const void *const info)
{
    for (size_t i = 0; i < g_event_count; ++i) {
        if (g_events[i].event == event) {
            g_events[i].callback(event, info, g_events[i].context);
        }
    }
}

/* ---------------------------------------------------------------------- */
/* Synthetic frame                                                         */
/* ---------------------------------------------------------------------- */

/*
 * What a channel should read this frame. Anything not named here gets a
 * type-appropriate default, which is enough to prove the registration works
 * without listing eighty channels.
 */
static bool override_value(const ChannelReg &r, scs_value_t &out)
{
    const char *n = r.name;

    if (strcmp(n, SCS_TELEMETRY_TRUCK_CHANNEL_speed) == 0) {
        out.type = SCS_VALUE_TYPE_float;
        out.value_float.value = 25.0f; /* 90 km/h */
        return true;
    }
    if (strcmp(n, SCS_TELEMETRY_TRUCK_CHANNEL_engine_rpm) == 0) {
        out.type = SCS_VALUE_TYPE_float;
        out.value_float.value = 1500.0f;
        return true;
    }
    if (strcmp(n, SCS_TELEMETRY_TRUCK_CHANNEL_fuel) == 0) {
        out.type = SCS_VALUE_TYPE_float;
        out.value_float.value = 350.0f; /* half of a 700 l tank */
        return true;
    }
    if (strcmp(n, SCS_TELEMETRY_TRUCK_CHANNEL_water_temperature) == 0) {
        out.type = SCS_VALUE_TYPE_float;
        out.value_float.value = 88.0f;
        return true;
    }
    if (strcmp(n, SCS_TELEMETRY_TRUCK_CHANNEL_effective_throttle) == 0) {
        out.type = SCS_VALUE_TYPE_float;
        out.value_float.value = 0.8f;
        return true;
    }
    if (strcmp(n, SCS_TELEMETRY_TRUCK_CHANNEL_wheel_velocity) == 0) {
        /* Rotations per second, not radians - see cluster_shm.h. A 0.5 m
           wheel covers 2*pi*0.5 = 3.1416 m per turn, so 25 m/s of road is
           7.9577 rev/s. The driven pair (2 and 3, per the config below)
           turn a quarter faster than that. */
        const float free = 25.0f / (2.0f * 3.14159265f * 0.5f);
        out.type = SCS_VALUE_TYPE_float;
        out.value_float.value = (r.index == 2 || r.index == 3) ? free * 1.25f : free;
        return true;
    }
    if (strcmp(n, SCS_TELEMETRY_TRUCK_CHANNEL_engine_gear) == 0) {
        out.type = SCS_VALUE_TYPE_s32;
        out.value_s32.value = 6;
        return true;
    }
    if (strcmp(n, SCS_TELEMETRY_CHANNEL_game_time) == 0) {
        out.type = SCS_VALUE_TYPE_u32;
        out.value_u32.value = 7 * 60 + 42; /* 07:42 */
        return true;
    }
    if (strcmp(n, SCS_TELEMETRY_TRUCK_CHANNEL_electric_enabled) == 0 ||
        strcmp(n, SCS_TELEMETRY_TRUCK_CHANNEL_engine_enabled) == 0 ||
        strcmp(n, SCS_TELEMETRY_TRUCK_CHANNEL_light_low_beam) == 0 ||
        strcmp(n, SCS_TELEMETRY_TRUCK_CHANNEL_rblinker) == 0) {
        out.type = SCS_VALUE_TYPE_bool;
        out.value_bool.value = 1;
        return true;
    }
    return false;
}

static void pump_frame(const scs_timestamp_t t)
{
    scs_telemetry_frame_start_t start;
    memset(&start, 0, sizeof(start));
    start.render_time = t;
    start.simulation_time = t;
    start.paused_simulation_time = t;
    fire_event(SCS_TELEMETRY_EVENT_frame_start, &start);

    for (size_t i = 0; i < g_channel_count; ++i) {
        const ChannelReg &r = g_channels[i];
        scs_value_t v;
        memset(&v, 0, sizeof(v));
        if (!override_value(r, v)) {
            v.type = r.type; /* zero of the right type */
        }
        r.callback(r.name, r.index, &v, r.context);
    }

    fire_event(SCS_TELEMETRY_EVENT_frame_end, nullptr);
}

static void send_truck_config()
{
    /* A 6-wheeler with a 700 l tank, driven on axle 2. */
    static scs_named_value_t attrs[32];
    size_t n = 0;

    attrs[n].name = SCS_TELEMETRY_CONFIG_ATTRIBUTE_fuel_capacity;
    attrs[n].index = SCS_U32_NIL;
    attrs[n].value.type = SCS_VALUE_TYPE_float;
    attrs[n].value.value_float.value = 700.0f;
    ++n;

    attrs[n].name = SCS_TELEMETRY_CONFIG_ATTRIBUTE_rpm_limit;
    attrs[n].index = SCS_U32_NIL;
    attrs[n].value.type = SCS_VALUE_TYPE_float;
    attrs[n].value.value_float.value = 2500.0f;
    ++n;

    attrs[n].name = SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_count;
    attrs[n].index = SCS_U32_NIL;
    attrs[n].value.type = SCS_VALUE_TYPE_u32;
    attrs[n].value.value_u32.value = 6;
    ++n;

    attrs[n].name = SCS_TELEMETRY_CONFIG_ATTRIBUTE_brand;
    attrs[n].index = SCS_U32_NIL;
    attrs[n].value.type = SCS_VALUE_TYPE_string;
    attrs[n].value.value_string.value = "Scania";
    ++n;

    attrs[n].name = SCS_TELEMETRY_CONFIG_ATTRIBUTE_name;
    attrs[n].index = SCS_U32_NIL;
    attrs[n].value.type = SCS_VALUE_TYPE_string;
    attrs[n].value.value_string.value = "R 2016";
    ++n;

    for (scs_u32_t i = 0; i < 6; ++i) {
        attrs[n].name = SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_radius;
        attrs[n].index = i;
        attrs[n].value.type = SCS_VALUE_TYPE_float;
        attrs[n].value.value_float.value = 0.5f;
        ++n;

        attrs[n].name = SCS_TELEMETRY_CONFIG_ATTRIBUTE_wheel_powered;
        attrs[n].index = i;
        attrs[n].value.type = SCS_VALUE_TYPE_bool;
        attrs[n].value.value_bool.value = (i == 2 || i == 3) ? 1 : 0;
        ++n;
    }

    attrs[n].name = nullptr; /* the SDK terminates the list with a null name */

    scs_telemetry_configuration_t config;
    config.id = SCS_TELEMETRY_CONFIG_truck;
    config.attributes = attrs;
    fire_event(SCS_TELEMETRY_EVENT_configuration, &config);
}

/* ---------------------------------------------------------------------- */

static const cluster_shm_t *map_shm(int *fd_out)
{
    const int fd = shm_open(kTestShmName, O_RDONLY, 0);
    if (fd < 0) {
        return nullptr;
    }
    void *const base = mmap(nullptr, CLUSTER_SHM_SIZE, PROT_READ, MAP_SHARED, fd, 0);
    if (base == MAP_FAILED) {
        close(fd);
        return nullptr;
    }
    *fd_out = fd;
    return static_cast<const cluster_shm_t *>(base);
}

int main(int argc, char **argv)
{
    const char *const path = (argc > 1) ? argv[1] : "./plugins/cluster-telemetry.so";

    printf("host_test: %s\n", path);
    printf("  sizeof(cluster_shm_t) = %zu\n", sizeof(cluster_shm_t));
    CHECK(sizeof(cluster_shm_t) == 512, "struct is %zu bytes, expected 512",
          sizeof(cluster_shm_t));

    /*
     * This test loads the real plugin, so it uses the real block name - and
     * its shutdown unlinks it. Run it while the game is up and it deletes
     * the name out from under a game that is still happily writing to the
     * mapping, which looks exactly like the plugin having failed. Decline
     * instead, and say which it is.
     */
    /*
     * Publish to our own name, never the one a running game uses. This test
     * loads the real plugin, and the plugin's shutdown unlinks whatever it
     * opened - sharing the name would delete a live game's block mid-frame,
     * which looks exactly like the plugin having failed. The plugin reads
     * this variable; see shm_name() there.
     */
    setenv("CLUSTER_SHM_NAME", kTestShmName, 1);

    static const char *const kTestBlockPath = "/dev/shm" kTestShmName;
    if (access(kTestBlockPath, F_OK) == 0) {
        printf("\n%s is left over from a previous run - removing it\n", kTestBlockPath);
        unlink(kTestBlockPath);
    }

    void *const lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (!lib) {
        printf("  FAIL: dlopen: %s\n", dlerror());
        return 1;
    }

    typedef scs_result_t (*init_fn)(const scs_u32_t, const scs_telemetry_init_params_t *const);
    typedef void (*shutdown_fn)(void);

    init_fn init = reinterpret_cast<init_fn>(dlsym(lib, "scs_telemetry_init"));
    shutdown_fn shutdown = reinterpret_cast<shutdown_fn>(dlsym(lib, "scs_telemetry_shutdown"));
    CHECK(init != nullptr, "scs_telemetry_init not exported");
    CHECK(shutdown != nullptr, "scs_telemetry_shutdown not exported");
    if (!init || !shutdown) {
        return 1;
    }

    scs_telemetry_init_params_v100_t params;
    memset(&params, 0, sizeof(params));
    params.common.game_name = "Euro Truck Simulator 2";
    params.common.game_id = SCS_GAME_ID_EUT2;
    params.common.game_version = SCS_TELEMETRY_EUT2_GAME_VERSION_CURRENT;
    params.common.log = fake_log;
    params.register_for_event = fake_register_event;
    params.unregister_from_event = fake_unregister_event;
    params.register_for_channel = fake_register_channel;
    params.unregister_from_channel = fake_unregister_channel;

    /* A version the plugin does not know must be declined, not guessed at. */
    CHECK(init(0x00090000, reinterpret_cast<const scs_telemetry_init_params_t *>(&params)) ==
              SCS_RESULT_unsupported,
          "an unknown API version was accepted");

    const scs_result_t rc =
        init(SCS_TELEMETRY_VERSION_1_01,
             reinterpret_cast<const scs_telemetry_init_params_t *>(&params));
    CHECK(rc == SCS_RESULT_ok, "init returned %d", rc);
    if (rc != SCS_RESULT_ok) {
        return 1;
    }

    printf("  registered %zu channels, %zu events\n", g_channel_count, g_event_count);
    CHECK(g_channel_count > 50, "only %zu channels registered", g_channel_count);
    CHECK(g_event_count == 5, "%zu events registered, expected 5", g_event_count);

    send_truck_config();
    pump_frame(1000000);
    pump_frame(2000000);

    int fd = -1;
    const cluster_shm_t *const shm = map_shm(&fd);
    if (!shm) {
        printf("  FAIL: nothing at " kTestShmName "\n");
        return 1;
    }

    /* Read under the seqlock, the way the Python reader does. */
    const uint32_t seq_before = shm->seq;
    cluster_shm_t snap;
    memcpy(&snap, shm, sizeof(snap));
    const uint32_t seq_after = shm->seq;

    CHECK(seq_before == seq_after, "seq moved during the read (%u -> %u)", seq_before,
          seq_after);
    CHECK((seq_before % 2) == 0, "seq %u is odd - a frame was left half written", seq_before);

    CHECK(snap.magic == CLUSTER_SHM_MAGIC, "magic is 0x%08x", snap.magic);
    CHECK(snap.version == CLUSTER_SHM_VERSION, "version is %u", snap.version);
    CHECK(snap.size == sizeof(cluster_shm_t), "size field is %u", snap.size);
    CHECK(snap.sdk_active == 1, "sdk_active is %u", snap.sdk_active);
    CHECK(snap.game == CLUSTER_GAME_ETS2, "game is %u", snap.game);
    CHECK(strcmp(snap.game_id, "eut2") == 0, "game_id is '%s'", snap.game_id);

    CHECK(snap.frames >= 3, "frames is %u, expected the config plus two frames",
          snap.frames);
    CHECK(snap.timestamp_us == 2000000, "timestamp_us is %llu",
          (unsigned long long)snap.timestamp_us);

    CHECK(fabsf(snap.speed - 25.0f) < 0.001f, "speed is %f", snap.speed);
    CHECK(fabsf(snap.engine_rpm - 1500.0f) < 0.001f, "rpm is %f", snap.engine_rpm);
    CHECK(fabsf(snap.fuel - 350.0f) < 0.001f, "fuel is %f", snap.fuel);
    CHECK(fabsf(snap.fuel_capacity - 700.0f) < 0.001f, "capacity is %f", snap.fuel_capacity);
    CHECK(fabsf(snap.engine_rpm_max - 2500.0f) < 0.001f, "rpm limit is %f",
          snap.engine_rpm_max);
    CHECK(fabsf(snap.water_temperature - 88.0f) < 0.001f, "coolant is %f",
          snap.water_temperature);
    CHECK(fabsf(snap.throttle - 0.8f) < 0.001f, "throttle is %f", snap.throttle);
    CHECK(snap.gear == 6, "gear is %d", snap.gear);
    CHECK(snap.time_abs == 7 * 60 + 42, "time_abs is %u", snap.time_abs);

    CHECK(snap.electric_enabled == 1, "electric_enabled is %u", snap.electric_enabled);
    CHECK(snap.engine_enabled == 1, "engine_enabled is %u", snap.engine_enabled);
    CHECK(snap.light_low_beam == 1, "low beam is %u", snap.light_low_beam);
    CHECK(snap.blinker_right_active == 1, "right blinker is %u", snap.blinker_right_active);
    CHECK(snap.blinker_left_active == 0, "left blinker is %u", snap.blinker_left_active);

    CHECK(snap.wheel_count == 6, "wheel_count is %u", snap.wheel_count);
    CHECK(strcmp(snap.truck_brand, "Scania") == 0, "brand is '%s'", snap.truck_brand);
    CHECK(strcmp(snap.truck_name, "R 2016") == 0, "name is '%s'", snap.truck_name);

    const float free_rev = 25.0f / (2.0f * 3.14159265f * 0.5f);
    bool wheels_ok = true;
    for (int i = 0; i < 6; ++i) {
        const bool powered = (i == 2 || i == 3);
        if (snap.wheel_powered[i] != (powered ? 1 : 0)) {
            wheels_ok = false;
        }
        if (fabsf(snap.wheel_radius[i] - 0.5f) > 0.001f) {
            wheels_ok = false;
        }
        const float want = powered ? free_rev * 1.25f : free_rev;
        if (fabsf(snap.wheel_angular_velocity[i] - want) > 0.001f) {
            wheels_ok = false;
        }
    }
    CHECK(wheels_ok, "wheel arrays did not come through");
    for (int i = 6; i < CLUSTER_WHEELS; ++i) {
        CHECK(snap.wheel_radius[i] == 0.0f, "wheel %d should be unused", i);
    }

    /* The same slip sum the host does, as a sanity check on the raw numbers.
       The 2*pi is the whole point: the channel is rotations per second, so a
       0.5 m wheel at 9.947 rev/s covers 31.25 m/s against 25 m/s of road.
       Without it this reads 4.97 m/s, slip comes out at -0.80, and the lamp
       never fires - which is exactly the bug this guards. */
    const float surface =
        snap.wheel_angular_velocity[2] * 2.0f * 3.14159265f * snap.wheel_radius[2];
    const float slip = (surface - snap.speed) / snap.speed;
    CHECK(fabsf(slip - 0.25f) < 0.01f, "slip works out at %f, expected 0.25", slip);

    /* --hold keeps publishing so another process - the Python reader - can
       check that it agrees with this struct byte for byte. Without it the
       two sides are only ever tested against their own idea of the layout,
       which is exactly the bug the seqlock and size fields exist to catch. */
    if (argc > 2 && strcmp(argv[2], "--hold") == 0) {
        const int seconds = (argc > 3) ? atoi(argv[3]) : 5;
        printf("  holding %d s, publishing frames at " kTestShmName "\n", seconds);
        fflush(stdout);
        for (int i = 0; i < seconds * 20; ++i) {
            pump_frame(3000000 + static_cast<scs_timestamp_t>(i) * 50000);
            usleep(50000);
        }
        printf("  held %d frames\n", snap.frames);
    }

    shutdown();

    /* Shutdown must stop advertising itself, or a reader would keep serving
       the last frame of a game that has quit. */
    CHECK(shm->magic == 0, "magic survived shutdown");
    munmap(const_cast<cluster_shm_t *>(shm), CLUSTER_SHM_SIZE);
    close(fd);

    CHECK(shm_open(kTestShmName, O_RDONLY, 0) < 0, "the block was left behind");

    dlclose(lib);

    if (g_failures) {
        printf("\n%d check(s) failed\n", g_failures);
        return 1;
    }
    printf("\nplugin selftest OK  (load, export, config, frame, seqlock, shutdown)\n");
    return 0;
}
