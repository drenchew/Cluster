/*
 * cluster_shm.h - the block the ETS2/ATS plugin publishes for the cluster host.
 *
 * This is the contract between two languages: the plugin writes this struct
 * into POSIX shared memory, host/cluster_sim/ets2_plugin.py reads it back with
 * ctypes. Both sides assert their own idea of the layout, so a field added on
 * one side only is a loud failure at startup rather than a needle that reads
 * 40 km/h too high.
 *
 * Rules when editing:
 *   - append, never insert, and bump CLUSTER_SHM_VERSION
 *   - keep every field naturally aligned (floats on 4, u64 on 8) so there is
 *     no implicit padding for ctypes to disagree about
 *   - the static_assert at the bottom is not decoration; keep it passing
 */

#ifndef CLUSTER_SHM_H
#define CLUSTER_SHM_H

#include <stdint.h>

/* shm_open() name. The kernel shows it as /dev/shm/ClusterTelemetry. */
#define CLUSTER_SHM_NAME "/ClusterTelemetry"

/* "CLM1" on a little-endian machine, which every ETS2 target is. */
#define CLUSTER_SHM_MAGIC 0x314D4C43u

/* Readers refuse a block whose major version they do not know. */
#define CLUSTER_SHM_VERSION 1u

/* One page. The struct is 512 bytes, so there is room to append for years
   without changing the size of the mapping. */
#define CLUSTER_SHM_SIZE 4096u

/* The SDK reports up to 16 wheels, and a truck with a heavy trailer axle
   configuration really does use double figures. */
#define CLUSTER_WHEELS 16

/* Values for cluster_shm_t.game. */
#define CLUSTER_GAME_UNKNOWN 0u
#define CLUSTER_GAME_ETS2 1u
#define CLUSTER_GAME_ATS 2u

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cluster_shm_t {
    /* ---- header (64 bytes) ------------------------------------------- */

    uint32_t magic;   /* CLUSTER_SHM_MAGIC once the plugin has initialised */
    uint32_t version; /* CLUSTER_SHM_VERSION */
    uint32_t size;    /* sizeof(cluster_shm_t), for a reader's sanity check */

    /* Seqlock. The plugin makes this odd before it copies a frame in and
       even again afterwards, so a reader that sees the same even value on
       both sides of its own read knows it did not catch a half-written
       frame. Everything else here is only meaningful under that check. */
    uint32_t seq;

    uint32_t game;        /* CLUSTER_GAME_* - which game is running */
    uint32_t game_major;  /* telemetry API version the game offers */
    uint32_t game_minor;
    uint32_t plugin_build; /* CLUSTER_PLUGIN_BUILD, bumped by hand */

    uint32_t sdk_active; /* 1 between scs_telemetry_init and shutdown */
    uint32_t paused;     /* 1 while the game is in a menu or loading */

    /* Minutes on the game's own calendar, which starts at an arbitrary
       epoch and runs several times faster than the wall clock. Only the
       time of day inside it is worth showing. */
    uint32_t time_abs;

    uint32_t wheel_count; /* how many entries of the wheel arrays are real */

    uint64_t timestamp_us; /* game simulation time, microseconds */

    uint32_t frames; /* frames published since init, a liveness counter */
    uint32_t _pad0;

    /* ---- floats ------------------------------------------------------ */

    float speed; /* m/s, negative while rolling backwards */
    float engine_rpm;
    float engine_rpm_max;
    float fuel;          /* litres in the tank */
    float fuel_capacity; /* litres the tank holds */
    float fuel_range;    /* km the game thinks is left */
    float fuel_avg_consumption;
    float water_temperature; /* degrees C */
    float oil_pressure;
    float oil_temperature;
    float brake_air_pressure;
    float battery_voltage;
    float throttle; /* 0..1, after the game's own smoothing */
    float brake;
    float clutch;
    float steering; /* -1..1 */
    float odometer; /* km */
    float cruise_control; /* m/s, 0 when off */
    float dashboard_backlight; /* 0..1 */
    float adblue;
    float wear_engine; /* 0..1 */
    float wear_transmission;
    float navigation_speed_limit; /* m/s, 0 when unknown */
    float navigation_distance;    /* m to the next waypoint */
    float navigation_time;        /* s to the next waypoint */
    float _pad1;

    /* ---- integers ---------------------------------------------------- */

    int32_t gear;           /* <0 reverse, 0 neutral, >0 forward */
    int32_t displayed_gear; /* what the truck's own dash shows */
    int32_t gears_forward;  /* how many forward gears the box has */
    int32_t hshifter_slot;

    /* ---- flags, one byte each --------------------------------------- */

    uint8_t electric_enabled; /* ignition - the cluster's KL15 */
    uint8_t engine_enabled;
    uint8_t park_brake;
    uint8_t motor_brake;

    uint8_t light_parking; /* side lights */
    uint8_t light_low_beam;
    uint8_t light_high_beam;
    uint8_t light_beacon;
    uint8_t light_brake;
    uint8_t light_reverse;

    /* The *_active pair is the stalk position, the *_on pair is the lamp
       at this instant. The cluster does its own flashing from 0x1F6, so the
       host wants the stalk, not the blink. */
    uint8_t blinker_left_active;
    uint8_t blinker_right_active;
    uint8_t blinker_left_on;
    uint8_t blinker_right_on;
    uint8_t hazard;

    uint8_t differential_lock;
    uint8_t cruise_control_on;

    uint8_t fuel_warning;
    uint8_t adblue_warning;
    uint8_t oil_pressure_warning;
    uint8_t water_temperature_warning;
    uint8_t battery_voltage_warning;
    uint8_t air_pressure_warning;
    uint8_t air_pressure_emergency;

    /* ---- per-wheel arrays ------------------------------------------- */

    /* ROTATIONS per second, not radians - the channel is named
       "angular_velocity" but the SDK header says rotations, and the game
       agrees: a free-rolling 0.506 m wheel at 91.6 km/h reports 8.005, and
       25.44 / (2*pi * 0.506) is 8.00.

       So surface speed is value * 2*pi * radius, which against the truck's
       ground speed is wheelspin - the only way the game will tell you a
       traction event is happening. Drop the 2*pi and surface speed comes out
       at 16% of the truth, slip goes permanently negative, and the lamp
       never lights. */
    float wheel_angular_velocity[CLUSTER_WHEELS];
    float wheel_radius[CLUSTER_WHEELS]; /* metres, from the truck config */

    uint8_t wheel_powered[CLUSTER_WHEELS];
    uint8_t wheel_on_ground[CLUSTER_WHEELS];
    uint8_t wheel_steerable[CLUSTER_WHEELS];
    uint8_t wheel_simulated[CLUSTER_WHEELS];

    /* ---- strings, NUL terminated ------------------------------------ */

    char truck_brand[48];
    char truck_name[48];
    char game_id[16]; /* "eut2" or "ats" */
} cluster_shm_t;

#ifdef __cplusplus
} /* extern "C" */
#endif

/* 512 bytes. If this fires, a field was inserted rather than appended, or
   one was added without minding alignment - fix the struct, do not widen
   the number, because the Python side checks the same figure. */
#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(sizeof(cluster_shm_t) == 512, "cluster_shm_t must stay 512 bytes");
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(cluster_shm_t) == 512, "cluster_shm_t must stay 512 bytes");
#endif

#endif /* CLUSTER_SHM_H */
