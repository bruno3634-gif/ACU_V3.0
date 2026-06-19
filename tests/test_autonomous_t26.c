/**
 * @file test_autonomous_t26.c
 *
 * @brief Standalone host-based unit test for the CAN DBC pack/unpack codec
 *        in autonomous_t26.c (cantools-generated).
 *
 * Compile with:
 *   gcc -o test_autonomous_t26 test_autonomous_t26.c -Wall -Wextra -std=c11 -lm
 *
 * @copyright Copyright (c) 2018-2019 Erik Moqvist (MIT License)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

/*===========================================================================*/
/* Standalone EINVAL -- errno.h replacement for host test                    */
/*===========================================================================*/
#ifndef EINVAL
#    define EINVAL 22
#endif

/*===========================================================================*/
/* Custom assert macro -- returns 0 on failure                               */
/*===========================================================================*/
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do {                                         \
    if (!(cond)) {                                                          \
        fprintf(stderr, "  FAIL [line %d]: %s\n", __LINE__, msg);           \
        tests_failed++;                                                     \
        return 0;                                                           \
    } else {                                                                \
        tests_passed++;                                                     \
    }                                                                       \
} while (0)

/*===========================================================================*/
/* Struct definitions (from autonomous_t26.h)                               */
/*===========================================================================*/

/**
 * Signals in message ACU.
 *
 * All signal values are as on the CAN bus.
 */
struct autonomous_t26_acu_t {
    uint8_t assi_state;
    uint8_t acu_state;
    uint8_t acu_cpu_temp;
    uint8_t mission_select;
    uint8_t as_state;
    uint8_t emergency;
    uint8_t asms;
    uint8_t ign;
    uint8_t emergency_cause;
};

/**
 * Signals in message AQT7.
 *
 * All signal values are as on the CAN bus.
 */
struct autonomous_t26_aqt7_t {
    uint16_t rear_brk_press;
};

/**
 * Signals in message VCU_IGN_R2D.
 *
 * All signal values are as on the CAN bus.
 */
struct autonomous_t26_vcu_ign_r2_d_t {
    uint8_t ignition_manual;
    uint8_t r2d_manual;
    uint8_t ignition_auto;
    uint8_t r2d_auto;
    uint8_t shutdown_signal;
    uint8_t vcu_state;
    uint8_t r2_d_button_raw;
    uint8_t ignition_switch_raw;
};

/**
 * Signals in message ASF_SIGNALS.
 *
 * All signal values are as on the CAN bus.
 */
struct autonomous_t26_asf_signals_t {
    uint8_t ebs_pressure_tank_front;
    uint8_t ebs_pressure_tank_rear;
    uint8_t brake_pressure_front;
    uint8_t brake_pressure_rear;
};

/**
 * Signals in message VCU_RPM.
 *
 * All signal values are as on the CAN bus.
 */
struct autonomous_t26_vcu_rpm_t {
    int16_t rpm_actual;
};

/*===========================================================================*/
/* Helper functions (copied verbatim from autonomous_t26.c)                 */
/*===========================================================================*/

static inline uint8_t pack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u32(
    uint32_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint8_t pack_right_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint8_t pack_right_shift_u32(
    uint32_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint8_t unpack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) << shift);
}

static inline uint16_t unpack_left_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint32_t unpack_left_shift_u32(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint32_t)((uint32_t)(value & mask) << shift);
}

static inline uint8_t unpack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) >> shift);
}

static inline uint16_t unpack_right_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

static inline uint32_t unpack_right_shift_u32(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint32_t)((uint32_t)(value & mask) >> shift);
}

/*===========================================================================*/
/* ACU message functions                                                    */
/*===========================================================================*/

int autonomous_t26_acu_pack(
    uint8_t *dst_p,
    const struct autonomous_t26_acu_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->assi_state, 0u, 0x0fu);
    dst_p[0] |= pack_left_shift_u8(src_p->acu_state, 4u, 0xf0u);
    dst_p[1] |= pack_left_shift_u8(src_p->acu_cpu_temp, 0u, 0xffu);
    dst_p[2] |= pack_left_shift_u8(src_p->mission_select, 0u, 0x07u);
    dst_p[2] |= pack_left_shift_u8(src_p->as_state, 3u, 0x38u);
    dst_p[2] |= pack_left_shift_u8(src_p->emergency, 6u, 0x40u);
    dst_p[2] |= pack_left_shift_u8(src_p->asms, 7u, 0x80u);
    dst_p[3] |= pack_left_shift_u8(src_p->ign, 0u, 0x01u);
    dst_p[3] |= pack_left_shift_u8(src_p->emergency_cause, 1u, 0xfeu);

    return (8);
}

int autonomous_t26_acu_unpack(
    struct autonomous_t26_acu_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->assi_state = unpack_right_shift_u8(src_p[0], 0u, 0x0fu);
    dst_p->acu_state = unpack_right_shift_u8(src_p[0], 4u, 0xf0u);
    dst_p->acu_cpu_temp = unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    dst_p->mission_select = unpack_right_shift_u8(src_p[2], 0u, 0x07u);
    dst_p->as_state = unpack_right_shift_u8(src_p[2], 3u, 0x38u);
    dst_p->emergency = unpack_right_shift_u8(src_p[2], 6u, 0x40u);
    dst_p->asms = unpack_right_shift_u8(src_p[2], 7u, 0x80u);
    dst_p->ign = unpack_right_shift_u8(src_p[3], 0u, 0x01u);
    dst_p->emergency_cause = unpack_right_shift_u8(src_p[3], 1u, 0xfeu);

    return (0);
}

int autonomous_t26_acu_init(struct autonomous_t26_acu_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct autonomous_t26_acu_t));

    return 0;
}

uint8_t autonomous_t26_acu_assi_state_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_acu_assi_state_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_acu_assi_state_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

uint8_t autonomous_t26_acu_acu_state_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_acu_acu_state_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_acu_acu_state_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

uint8_t autonomous_t26_acu_acu_cpu_temp_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_acu_acu_cpu_temp_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_acu_acu_cpu_temp_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t autonomous_t26_acu_mission_select_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_acu_mission_select_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_acu_mission_select_is_in_range(uint8_t value)
{
    return (value <= 7u);
}

uint8_t autonomous_t26_acu_as_state_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_acu_as_state_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_acu_as_state_is_in_range(uint8_t value)
{
    return (value <= 7u);
}

uint8_t autonomous_t26_acu_emergency_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_acu_emergency_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_acu_emergency_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t autonomous_t26_acu_asms_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_acu_asms_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_acu_asms_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t autonomous_t26_acu_ign_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_acu_ign_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_acu_ign_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t autonomous_t26_acu_emergency_cause_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_acu_emergency_cause_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_acu_emergency_cause_is_in_range(uint8_t value)
{
    return (value <= 127u);
}

/*===========================================================================*/
/* AQT7 message functions                                                   */
/*===========================================================================*/

int autonomous_t26_aqt7_pack(
    uint8_t *dst_p,
    const struct autonomous_t26_aqt7_t *src_p,
    size_t size)
{
    if (size < 2u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 2);

    dst_p[0] |= pack_left_shift_u16(src_p->rear_brk_press, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(src_p->rear_brk_press, 8u, 0xffu);

    return (2);
}

int autonomous_t26_aqt7_unpack(
    struct autonomous_t26_aqt7_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 2u) {
        return (-EINVAL);
    }

    dst_p->rear_brk_press = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    dst_p->rear_brk_press |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);

    return (0);
}

int autonomous_t26_aqt7_init(struct autonomous_t26_aqt7_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct autonomous_t26_aqt7_t));

    return 0;
}

uint16_t autonomous_t26_aqt7_rear_brk_press_encode(double value)
{
    return (uint16_t)(value / 0.1);
}

double autonomous_t26_aqt7_rear_brk_press_decode(uint16_t value)
{
    return ((double)value * 0.1);
}

bool autonomous_t26_aqt7_rear_brk_press_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

/*===========================================================================*/
/* VCU_IGN_R2_D message functions                                           */
/*===========================================================================*/

int autonomous_t26_vcu_ign_r2_d_pack(
    uint8_t *dst_p,
    const struct autonomous_t26_vcu_ign_r2_d_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->ignition_manual, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u8(src_p->r2d_manual, 0u, 0xffu);
    dst_p[2] |= pack_left_shift_u8(src_p->ignition_auto, 0u, 0xffu);
    dst_p[3] |= pack_left_shift_u8(src_p->r2d_auto, 0u, 0xffu);
    dst_p[4] |= pack_left_shift_u8(src_p->shutdown_signal, 0u, 0xffu);
    dst_p[5] |= pack_left_shift_u8(src_p->vcu_state, 0u, 0xffu);
    dst_p[6] |= pack_left_shift_u8(src_p->r2_d_button_raw, 0u, 0xffu);
    dst_p[7] |= pack_left_shift_u8(src_p->ignition_switch_raw, 0u, 0xffu);

    return (8);
}

int autonomous_t26_vcu_ign_r2_d_unpack(
    struct autonomous_t26_vcu_ign_r2_d_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->ignition_manual = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->r2d_manual = unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    dst_p->ignition_auto = unpack_right_shift_u8(src_p[2], 0u, 0xffu);
    dst_p->r2d_auto = unpack_right_shift_u8(src_p[3], 0u, 0xffu);
    dst_p->shutdown_signal = unpack_right_shift_u8(src_p[4], 0u, 0xffu);
    dst_p->vcu_state = unpack_right_shift_u8(src_p[5], 0u, 0xffu);
    dst_p->r2_d_button_raw = unpack_right_shift_u8(src_p[6], 0u, 0xffu);
    dst_p->ignition_switch_raw = unpack_right_shift_u8(src_p[7], 0u, 0xffu);

    return (0);
}

int autonomous_t26_vcu_ign_r2_d_init(struct autonomous_t26_vcu_ign_r2_d_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct autonomous_t26_vcu_ign_r2_d_t));

    return 0;
}

uint8_t autonomous_t26_vcu_ign_r2_d_ignition_manual_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_vcu_ign_r2_d_ignition_manual_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_vcu_ign_r2_d_ignition_manual_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t autonomous_t26_vcu_ign_r2_d_r2d_manual_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_vcu_ign_r2_d_r2d_manual_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_vcu_ign_r2_d_r2d_manual_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t autonomous_t26_vcu_ign_r2_d_ignition_auto_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_vcu_ign_r2_d_ignition_auto_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_vcu_ign_r2_d_ignition_auto_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t autonomous_t26_vcu_ign_r2_d_r2d_auto_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_vcu_ign_r2_d_r2d_auto_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_vcu_ign_r2_d_r2d_auto_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t autonomous_t26_vcu_ign_r2_d_shutdown_signal_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_vcu_ign_r2_d_shutdown_signal_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_vcu_ign_r2_d_shutdown_signal_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t autonomous_t26_vcu_ign_r2_d_vcu_state_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_vcu_ign_r2_d_vcu_state_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_vcu_ign_r2_d_vcu_state_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t autonomous_t26_vcu_ign_r2_d_r2_d_button_raw_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_vcu_ign_r2_d_r2_d_button_raw_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_vcu_ign_r2_d_r2_d_button_raw_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t autonomous_t26_vcu_ign_r2_d_ignition_switch_raw_encode(double value)
{
    return (uint8_t)(value);
}

double autonomous_t26_vcu_ign_r2_d_ignition_switch_raw_decode(uint8_t value)
{
    return ((double)value);
}

bool autonomous_t26_vcu_ign_r2_d_ignition_switch_raw_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

/*===========================================================================*/
/* ASF_SIGNALS message functions                                            */
/*===========================================================================*/

int autonomous_t26_asf_signals_pack(
    uint8_t *dst_p,
    const struct autonomous_t26_asf_signals_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    dst_p[0] |= pack_left_shift_u8(src_p->ebs_pressure_tank_front, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u8(src_p->ebs_pressure_tank_rear, 0u, 0xffu);
    dst_p[2] |= pack_left_shift_u8(src_p->brake_pressure_front, 0u, 0xffu);
    dst_p[3] |= pack_left_shift_u8(src_p->brake_pressure_rear, 0u, 0xffu);

    return (4);
}

int autonomous_t26_asf_signals_unpack(
    struct autonomous_t26_asf_signals_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 4u) {
        return (-EINVAL);
    }

    dst_p->ebs_pressure_tank_front = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->ebs_pressure_tank_rear = unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    dst_p->brake_pressure_front = unpack_right_shift_u8(src_p[2], 0u, 0xffu);
    dst_p->brake_pressure_rear = unpack_right_shift_u8(src_p[3], 0u, 0xffu);

    return (0);
}

int autonomous_t26_asf_signals_init(struct autonomous_t26_asf_signals_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct autonomous_t26_asf_signals_t));

    return 0;
}

uint8_t autonomous_t26_asf_signals_ebs_pressure_tank_front_encode(double value)
{
    return (uint8_t)(value / 0.1);
}

double autonomous_t26_asf_signals_ebs_pressure_tank_front_decode(uint8_t value)
{
    return ((double)value * 0.1);
}

bool autonomous_t26_asf_signals_ebs_pressure_tank_front_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t autonomous_t26_asf_signals_ebs_pressure_tank_rear_encode(double value)
{
    return (uint8_t)(value / 0.1);
}

double autonomous_t26_asf_signals_ebs_pressure_tank_rear_decode(uint8_t value)
{
    return ((double)value * 0.1);
}

bool autonomous_t26_asf_signals_ebs_pressure_tank_rear_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t autonomous_t26_asf_signals_brake_pressure_front_encode(double value)
{
    return (uint8_t)(value / 0.1);
}

double autonomous_t26_asf_signals_brake_pressure_front_decode(uint8_t value)
{
    return ((double)value * 0.1);
}

bool autonomous_t26_asf_signals_brake_pressure_front_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t autonomous_t26_asf_signals_brake_pressure_rear_encode(double value)
{
    return (uint8_t)(value / 0.1);
}

double autonomous_t26_asf_signals_brake_pressure_rear_decode(uint8_t value)
{
    return ((double)value * 0.1);
}

bool autonomous_t26_asf_signals_brake_pressure_rear_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

/*===========================================================================*/
/* VCU_RPM message functions                                                */
/*===========================================================================*/

int autonomous_t26_vcu_rpm_pack(
    uint8_t *dst_p,
    const struct autonomous_t26_vcu_rpm_t *src_p,
    size_t size)
{
    uint16_t rpm_actual;

    if (size < 2u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 2);

    rpm_actual = (uint16_t)src_p->rpm_actual;
    dst_p[0] |= pack_left_shift_u16(rpm_actual, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(rpm_actual, 8u, 0xffu);

    return (2);
}

int autonomous_t26_vcu_rpm_unpack(
    struct autonomous_t26_vcu_rpm_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t rpm_actual;

    if (size < 2u) {
        return (-EINVAL);
    }

    rpm_actual = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    rpm_actual |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->rpm_actual = (int16_t)rpm_actual;

    return (0);
}

int autonomous_t26_vcu_rpm_init(struct autonomous_t26_vcu_rpm_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct autonomous_t26_vcu_rpm_t));

    return 0;
}

int16_t autonomous_t26_vcu_rpm_rpm_actual_encode(double value)
{
    return (int16_t)(value);
}

double autonomous_t26_vcu_rpm_rpm_actual_decode(int16_t value)
{
    return ((double)value);
}

bool autonomous_t26_vcu_rpm_rpm_actual_is_in_range(int16_t value)
{
    return ((value >= 0) && (value <= 6000));
}

/*===========================================================================*/
/* Test 1: ACU message round-trip                                           */
/*===========================================================================*/
static int test_acu_round_trip(void)
{
    struct autonomous_t26_acu_t src;
    struct autonomous_t26_acu_t dst;
    uint8_t buf[8];
    int ret;

    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    memset(buf, 0, sizeof(buf));

    /* Fill with known distinct values */
    src.assi_state = 3;
    src.acu_state = 5;
    src.acu_cpu_temp = 42;
    src.mission_select = 2;
    src.as_state = 4;
    src.emergency = 1;
    src.asms = 1;
    src.ign = 1;
    src.emergency_cause = 2;

    ret = autonomous_t26_acu_pack(buf, &src, sizeof(buf));
    TEST_ASSERT(ret == 8, "ACU pack should return 8");

    ret = autonomous_t26_acu_unpack(&dst, buf, sizeof(buf));
    TEST_ASSERT(ret == 0, "ACU unpack should return 0");

    TEST_ASSERT(dst.assi_state == 3,          "ACU assi_state round-trip");
    TEST_ASSERT(dst.acu_state == 5,           "ACU acu_state round-trip");
    TEST_ASSERT(dst.acu_cpu_temp == 42,       "ACU acu_cpu_temp round-trip");
    TEST_ASSERT(dst.mission_select == 2,      "ACU mission_select round-trip");
    TEST_ASSERT(dst.as_state == 4,            "ACU as_state round-trip");
    TEST_ASSERT(dst.emergency == 1,           "ACU emergency round-trip");
    TEST_ASSERT(dst.asms == 1,                "ACU asms round-trip");
    TEST_ASSERT(dst.ign == 1,                 "ACU ign round-trip");
    TEST_ASSERT(dst.emergency_cause == 2,     "ACU emergency_cause round-trip");

    return 1;
}

/*===========================================================================*/
/* Test 2: ACU buffer too small                                             */
/*===========================================================================*/
static int test_acu_buffer_too_small(void)
{
    struct autonomous_t26_acu_t src;
    uint8_t small_buf[5];
    int ret;

    memset(&src, 0, sizeof(src));
    memset(small_buf, 0, sizeof(small_buf));

    ret = autonomous_t26_acu_pack(small_buf, &src, 5);
    TEST_ASSERT(ret == -EINVAL, "ACU pack with size=5 should return -EINVAL");
    TEST_ASSERT(ret == -22, "ACU pack with size=5 should return -22");

    return 1;
}

/*===========================================================================*/
/* Test 3: AQT7 rear brake pressure round-trip                              */
/*===========================================================================*/
static int test_aqt7_round_trip(void)
{
    struct autonomous_t26_aqt7_t src;
    struct autonomous_t26_aqt7_t dst;
    uint8_t buf[2];
    int ret;

    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    memset(buf, 0, sizeof(buf));

    src.rear_brk_press = 1234;

    ret = autonomous_t26_aqt7_pack(buf, &src, sizeof(buf));
    TEST_ASSERT(ret == 2, "AQT7 pack should return 2");

    ret = autonomous_t26_aqt7_unpack(&dst, buf, sizeof(buf));
    TEST_ASSERT(ret == 0, "AQT7 unpack should return 0");

    TEST_ASSERT(dst.rear_brk_press == 1234, "AQT7 rear_brk_press round-trip");

    return 1;
}

/*===========================================================================*/
/* Test 4: VCU_IGN_R2_D round-trip                                          */
/*===========================================================================*/
static int test_vcu_ign_r2_d_round_trip(void)
{
    struct autonomous_t26_vcu_ign_r2_d_t src;
    struct autonomous_t26_vcu_ign_r2_d_t dst;
    uint8_t buf[8];
    int ret;

    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    memset(buf, 0, sizeof(buf));

    /* Fill all 8 fields with distinct values */
    src.ignition_manual = 1;
    src.r2d_manual = 0;
    src.ignition_auto = 1;
    src.r2d_auto = 1;
    src.shutdown_signal = 0;
    src.vcu_state = 3;
    src.r2_d_button_raw = 1;
    src.ignition_switch_raw = 1;

    ret = autonomous_t26_vcu_ign_r2_d_pack(buf, &src, sizeof(buf));
    TEST_ASSERT(ret == 8, "VCU_IGN_R2_D pack should return 8");

    ret = autonomous_t26_vcu_ign_r2_d_unpack(&dst, buf, sizeof(buf));
    TEST_ASSERT(ret == 0, "VCU_IGN_R2_D unpack should return 0");

    TEST_ASSERT(dst.ignition_manual == 1,    "IGN_R2_D ignition_manual");
    TEST_ASSERT(dst.r2d_manual == 0,         "IGN_R2_D r2d_manual");
    TEST_ASSERT(dst.ignition_auto == 1,      "IGN_R2_D ignition_auto");
    TEST_ASSERT(dst.r2d_auto == 1,           "IGN_R2_D r2d_auto");
    TEST_ASSERT(dst.shutdown_signal == 0,    "IGN_R2_D shutdown_signal");
    TEST_ASSERT(dst.vcu_state == 3,          "IGN_R2_D vcu_state");
    TEST_ASSERT(dst.r2_d_button_raw == 1,    "IGN_R2_D r2_d_button_raw");
    TEST_ASSERT(dst.ignition_switch_raw == 1,"IGN_R2_D ignition_switch_raw");

    return 1;
}

/*===========================================================================*/
/* Test 5: VCU_RPM round-trip with value 3000                               */
/*===========================================================================*/
static int test_vcu_rpm_round_trip(void)
{
    struct autonomous_t26_vcu_rpm_t src;
    struct autonomous_t26_vcu_rpm_t dst;
    uint8_t buf[2];
    int ret;

    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    memset(buf, 0, sizeof(buf));

    src.rpm_actual = 3000;

    ret = autonomous_t26_vcu_rpm_pack(buf, &src, sizeof(buf));
    TEST_ASSERT(ret == 2, "VCU_RPM pack should return 2");

    ret = autonomous_t26_vcu_rpm_unpack(&dst, buf, sizeof(buf));
    TEST_ASSERT(ret == 0, "VCU_RPM unpack should return 0");

    TEST_ASSERT(dst.rpm_actual == 3000, "VCU_RPM rpm_actual round-trip (3000)");

    return 1;
}

/*===========================================================================*/
/* Test 6: ASF_SIGNALS round-trip                                           */
/*===========================================================================*/
static int test_asf_signals_round_trip(void)
{
    struct autonomous_t26_asf_signals_t src;
    struct autonomous_t26_asf_signals_t dst;
    uint8_t buf[4];
    int ret;

    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));
    memset(buf, 0, sizeof(buf));

    src.ebs_pressure_tank_front = 50;
    src.ebs_pressure_tank_rear = 60;
    src.brake_pressure_front = 70;
    src.brake_pressure_rear = 80;

    ret = autonomous_t26_asf_signals_pack(buf, &src, sizeof(buf));
    TEST_ASSERT(ret == 4, "ASF_SIGNALS pack should return 4");

    ret = autonomous_t26_asf_signals_unpack(&dst, buf, sizeof(buf));
    TEST_ASSERT(ret == 0, "ASF_SIGNALS unpack should return 0");

    TEST_ASSERT(dst.ebs_pressure_tank_front == 50, "ASF ebs_pressure_tank_front");
    TEST_ASSERT(dst.ebs_pressure_tank_rear == 60,  "ASF ebs_pressure_tank_rear");
    TEST_ASSERT(dst.brake_pressure_front == 70,    "ASF brake_pressure_front");
    TEST_ASSERT(dst.brake_pressure_rear == 80,     "ASF brake_pressure_rear");

    return 1;
}

/*===========================================================================*/
/* Test 7: ASF_SIGNALS buffer overflow                                      */
/*===========================================================================*/
static int test_asf_signals_buffer_overflow(void)
{
    struct autonomous_t26_asf_signals_t src;
    uint8_t small_buf[2];
    int ret;

    memset(&src, 0, sizeof(src));
    memset(small_buf, 0, sizeof(small_buf));

    ret = autonomous_t26_asf_signals_pack(small_buf, &src, 2);
    TEST_ASSERT(ret == -EINVAL, "ASF_SIGNALS pack with size=2 should return -EINVAL");
    TEST_ASSERT(ret == -22, "ASF_SIGNALS pack with size=2 should return -22");

    return 1;
}

/*===========================================================================*/
/* Test 8: init functions zero structs                                      */
/*===========================================================================*/
static int test_init_functions(void)
{
    int ret;

    /* ACU init */
    {
        struct autonomous_t26_acu_t msg;
        memset(&msg, 0xFF, sizeof(msg));
        ret = autonomous_t26_acu_init(&msg);
        TEST_ASSERT(ret == 0, "ACU init returned 0");
        TEST_ASSERT(msg.assi_state == 0,       "ACU init: assi_state == 0");
        TEST_ASSERT(msg.acu_state == 0,        "ACU init: acu_state == 0");
        TEST_ASSERT(msg.acu_cpu_temp == 0,     "ACU init: acu_cpu_temp == 0");
        TEST_ASSERT(msg.mission_select == 0,   "ACU init: mission_select == 0");
        TEST_ASSERT(msg.as_state == 0,         "ACU init: as_state == 0");
        TEST_ASSERT(msg.emergency == 0,        "ACU init: emergency == 0");
        TEST_ASSERT(msg.asms == 0,             "ACU init: asms == 0");
        TEST_ASSERT(msg.ign == 0,              "ACU init: ign == 0");
        TEST_ASSERT(msg.emergency_cause == 0,  "ACU init: emergency_cause == 0");
    }

    /* AQT7 init */
    {
        struct autonomous_t26_aqt7_t msg;
        memset(&msg, 0xFF, sizeof(msg));
        ret = autonomous_t26_aqt7_init(&msg);
        TEST_ASSERT(ret == 0, "AQT7 init returned 0");
        TEST_ASSERT(msg.rear_brk_press == 0, "AQT7 init: rear_brk_press == 0");
    }

    /* VCU_IGN_R2_D init */
    {
        struct autonomous_t26_vcu_ign_r2_d_t msg;
        memset(&msg, 0xFF, sizeof(msg));
        ret = autonomous_t26_vcu_ign_r2_d_init(&msg);
        TEST_ASSERT(ret == 0, "VCU_IGN_R2_D init returned 0");
        TEST_ASSERT(msg.ignition_manual == 0,    "IGN_R2_D init: ignition_manual == 0");
        TEST_ASSERT(msg.r2d_manual == 0,         "IGN_R2_D init: r2d_manual == 0");
        TEST_ASSERT(msg.ignition_auto == 0,      "IGN_R2_D init: ignition_auto == 0");
        TEST_ASSERT(msg.r2d_auto == 0,           "IGN_R2_D init: r2d_auto == 0");
        TEST_ASSERT(msg.shutdown_signal == 0,    "IGN_R2_D init: shutdown_signal == 0");
        TEST_ASSERT(msg.vcu_state == 0,          "IGN_R2_D init: vcu_state == 0");
        TEST_ASSERT(msg.r2_d_button_raw == 0,    "IGN_R2_D init: r2_d_button_raw == 0");
        TEST_ASSERT(msg.ignition_switch_raw == 0,"IGN_R2_D init: ignition_switch_raw == 0");
    }

    /* VCU_RPM init */
    {
        struct autonomous_t26_vcu_rpm_t msg;
        memset(&msg, 0xFF, sizeof(msg));
        ret = autonomous_t26_vcu_rpm_init(&msg);
        TEST_ASSERT(ret == 0, "VCU_RPM init returned 0");
        TEST_ASSERT(msg.rpm_actual == 0, "VCU_RPM init: rpm_actual == 0");
    }

    /* ASF_SIGNALS init */
    {
        struct autonomous_t26_asf_signals_t msg;
        memset(&msg, 0xFF, sizeof(msg));
        ret = autonomous_t26_asf_signals_init(&msg);
        TEST_ASSERT(ret == 0, "ASF_SIGNALS init returned 0");
        TEST_ASSERT(msg.ebs_pressure_tank_front == 0, "ASF init: ebs_pressure_tank_front == 0");
        TEST_ASSERT(msg.ebs_pressure_tank_rear == 0,  "ASF init: ebs_pressure_tank_rear == 0");
        TEST_ASSERT(msg.brake_pressure_front == 0,    "ASF init: brake_pressure_front == 0");
        TEST_ASSERT(msg.brake_pressure_rear == 0,     "ASF init: brake_pressure_rear == 0");
    }

    return 1;
}

/*===========================================================================*/
/* Test 9: ACU signal encode/decode with known values                       */
/*===========================================================================*/
static int test_acu_signal_encode_decode(void)
{
    /* Test assi_state: encode 3 -> should be 3, decode -> 3.0 */
    {
        uint8_t enc = autonomous_t26_acu_assi_state_encode(3.0);
        TEST_ASSERT(enc == 3, "assi_state_encode(3.0) == 3");

        double dec = autonomous_t26_acu_assi_state_decode(3);
        TEST_ASSERT(fabs(dec - 3.0) < 1e-9, "assi_state_decode(3) == 3.0");

        bool in_range = autonomous_t26_acu_assi_state_is_in_range(3);
        TEST_ASSERT(in_range == true, "assi_state 3 is in range");

        in_range = autonomous_t26_acu_assi_state_is_in_range(16);
        TEST_ASSERT(in_range == false, "assi_state 16 is out of range");
    }

    /* Test acu_cpu_temp: encode 42 -> should be 42, decode -> 42.0 */
    {
        uint8_t enc = autonomous_t26_acu_acu_cpu_temp_encode(42.0);
        TEST_ASSERT(enc == 42, "acu_cpu_temp_encode(42.0) == 42");

        double dec = autonomous_t26_acu_acu_cpu_temp_decode(42);
        TEST_ASSERT(fabs(dec - 42.0) < 1e-9, "acu_cpu_temp_decode(42) == 42.0");

        bool in_range = autonomous_t26_acu_acu_cpu_temp_is_in_range(42);
        TEST_ASSERT(in_range == true, "acu_cpu_temp 42 is in range");
    }

    /* Test acu_state: encode 5 -> 5, decode -> 5.0 */
    {
        uint8_t enc = autonomous_t26_acu_acu_state_encode(5.0);
        TEST_ASSERT(enc == 5, "acu_state_encode(5.0) == 5");

        double dec = autonomous_t26_acu_acu_state_decode(5);
        TEST_ASSERT(fabs(dec - 5.0) < 1e-9, "acu_state_decode(5) == 5.0");

        bool in_range = autonomous_t26_acu_acu_state_is_in_range(15);
        TEST_ASSERT(in_range == true, "acu_state 15 is in range");

        in_range = autonomous_t26_acu_acu_state_is_in_range(16);
        TEST_ASSERT(in_range == false, "acu_state 16 is out of range");
    }

    /* Test mission_select: encode 2 -> 2, decode -> 2.0 */
    {
        uint8_t enc = autonomous_t26_acu_mission_select_encode(2.0);
        TEST_ASSERT(enc == 2, "mission_select_encode(2.0) == 2");

        double dec = autonomous_t26_acu_mission_select_decode(2);
        TEST_ASSERT(fabs(dec - 2.0) < 1e-9, "mission_select_decode(2) == 2.0");

        bool in_range = autonomous_t26_acu_mission_select_is_in_range(7);
        TEST_ASSERT(in_range == true, "mission_select 7 is in range");

        in_range = autonomous_t26_acu_mission_select_is_in_range(8);
        TEST_ASSERT(in_range == false, "mission_select 8 is out of range");
    }

    /* Test as_state: encode 4 -> 4, decode -> 4.0 */
    {
        uint8_t enc = autonomous_t26_acu_as_state_encode(4.0);
        TEST_ASSERT(enc == 4, "as_state_encode(4.0) == 4");
    }

    /* Test emergency: encode 1 -> 1, decode -> 1.0 */
    {
        uint8_t enc = autonomous_t26_acu_emergency_encode(1.0);
        TEST_ASSERT(enc == 1, "emergency_encode(1.0) == 1");

        bool in_range = autonomous_t26_acu_emergency_is_in_range(0);
        TEST_ASSERT(in_range == true, "emergency 0 is in range");
        in_range = autonomous_t26_acu_emergency_is_in_range(1);
        TEST_ASSERT(in_range == true, "emergency 1 is in range");
        in_range = autonomous_t26_acu_emergency_is_in_range(2);
        TEST_ASSERT(in_range == false, "emergency 2 is out of range");
    }

    /* Test asms: encode 1 -> 1 */
    {
        uint8_t enc = autonomous_t26_acu_asms_encode(1.0);
        TEST_ASSERT(enc == 1, "asms_encode(1.0) == 1");
    }

    /* Test ign: encode 1 -> 1 */
    {
        uint8_t enc = autonomous_t26_acu_ign_encode(1.0);
        TEST_ASSERT(enc == 1, "ign_encode(1.0) == 1");
    }

    /* Test emergency_cause: encode 2 -> 2, range check */
    {
        uint8_t enc = autonomous_t26_acu_emergency_cause_encode(2.0);
        TEST_ASSERT(enc == 2, "emergency_cause_encode(2.0) == 2");

        bool in_range = autonomous_t26_acu_emergency_cause_is_in_range(127);
        TEST_ASSERT(in_range == true, "emergency_cause 127 is in range");
        in_range = autonomous_t26_acu_emergency_cause_is_in_range(128);
        TEST_ASSERT(in_range == false, "emergency_cause 128 is out of range");
    }

    return 1;
}

/*===========================================================================*/
/* Main: run all tests, count results                                       */
/*===========================================================================*/
int main(void)
{
    int total = 0;
    int passed = 0;

    printf("=== autonomous_t26 CAN codec unit tests ===\n\n");

    /* Run each test and accumulate counts */
    tests_passed = 0;
    tests_failed = 0;

    /* Test 1 */
    printf("TEST 1: ACU message round-trip...\n");
    if (test_acu_round_trip()) { /* pass */ }
    printf("  -> %s\n", (tests_failed > 0) ? "FAIL" : "PASS");
    total++;

    /* Test 2 */
    printf("TEST 2: ACU buffer too small (size=5)...\n");
    if (test_acu_buffer_too_small()) { /* pass */ }
    printf("  -> %s\n", (tests_failed > 0) ? "FAIL" : "PASS");
    total++;

    /* Test 3 */
    printf("TEST 3: AQT7 rear brake pressure round-trip...\n");
    if (test_aqt7_round_trip()) { /* pass */ }
    printf("  -> %s\n", (tests_failed > 0) ? "FAIL" : "PASS");
    total++;

    /* Test 4 */
    printf("TEST 4: VCU_IGN_R2_D round-trip...\n");
    if (test_vcu_ign_r2_d_round_trip()) { /* pass */ }
    printf("  -> %s\n", (tests_failed > 0) ? "FAIL" : "PASS");
    total++;

    /* Test 5 */
    printf("TEST 5: VCU_RPM round-trip (3000 RPM)...\n");
    if (test_vcu_rpm_round_trip()) { /* pass */ }
    printf("  -> %s\n", (tests_failed > 0) ? "FAIL" : "PASS");
    total++;

    /* Test 6 */
    printf("TEST 6: ASF_SIGNALS round-trip (4 fields)...\n");
    if (test_asf_signals_round_trip()) { /* pass */ }
    printf("  -> %s\n", (tests_failed > 0) ? "FAIL" : "PASS");
    total++;

    /* Test 7 */
    printf("TEST 7: ASF_SIGNALS buffer overflow (size=2)...\n");
    if (test_asf_signals_buffer_overflow()) { /* pass */ }
    printf("  -> %s\n", (tests_failed > 0) ? "FAIL" : "PASS");
    total++;

    /* Test 8 */
    printf("TEST 8: init functions zero structs...\n");
    if (test_init_functions()) { /* pass */ }
    printf("  -> %s\n", (tests_failed > 0) ? "FAIL" : "PASS");
    total++;

    /* Test 9 */
    printf("TEST 9: ACU signal encode/decode (acu_cpu_temp=42)...\n");
    if (test_acu_signal_encode_decode()) { /* pass */ }
    printf("  -> %s\n", (tests_failed > 0) ? "FAIL" : "PASS");
    total++;

    printf("\n=== Results: %d / %d tests passed ===\n",
           total - tests_failed, total);

    return (tests_failed > 0) ? 1 : 0;
}
