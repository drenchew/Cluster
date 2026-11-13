/**
 * @file cluster_shm.h
 * @brief Layout of the block published by the telemetry plugin.
 *
 * This header is the contract between two languages. The plugin writes the
 * structure into POSIX shared memory and host/cluster_sim/ets2_plugin.py
 * reads it back through ctypes. Each side asserts its own view of the size,
 * so a field added to one and not the other fails at startup rather than
 * silently shifting every value that follows it.
 *
 * Rules for modification:
 *   - append fields, never insert, and increment CLUSTER_SHM_VERSION
 *   - keep every member naturally aligned so that no implicit padding is
 *     introduced, since the two languages must agree on it
 *   - the static assertion at the end is part of that contract
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
       epoch and advances faster than real time, so only the time of day it
       represents is meaningful. */
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

       Surface speed is therefore value * 2*pi * radius. Compared against
       the vehicle's ground speed this gives wheel slip, which is the only
       indication the simulator provides that a traction event is occurring.
       Omitting the factor reduces surface speed to about 16 percent of its
       true value, which makes the computed slip permanently negative. */
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
