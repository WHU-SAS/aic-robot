#ifndef AIC_MOTOR_PROTOCOL_H
#define AIC_MOTOR_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* CRC-16/CCITT-FALSE: init FFFF, poly 1021, no reflection/xorout.
 * Request: @M,01234567,START,ABCD\n. CRC covers M,...,START only.
 * An '@' always resynchronizes, including inside an incomplete frame. */
#define MOTOR_PWM_LIMIT 1600
typedef enum { CMD_STATUS, CMD_START, CMD_STOP, CMD_HEARTBEAT, CMD_DRIVE, CMD_MOVE, CMD_TURN,
               CMD_FOLLOW, CMD_SPEED, CMD_ARC } MotorCommand;
typedef struct {
    char data[48];
    size_t used;
    bool active;
    uint32_t sequence;
    MotorCommand command;
    int16_t left_pwm, right_pwm;
    uint16_t duration_ticks;
    uint16_t distance_mm; /* Absolute distance; direction is separate. */
    bool reverse_move;
    int16_t angle_deg; /* Positive left, negative right; single-wheel forward arc. */
    uint16_t left_speed, right_speed; /* mm/s, physical forward is positive. */
    int16_t arc_left_mm, arc_right_mm; /* Signed relative wheel travel. */
} MotorParser;

static inline uint16_t motor_crc(const char *text, size_t size)
{
    uint16_t crc = 0xFFFF;
    while (size--) {
        crc ^= (uint16_t)(uint8_t)*text++ << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
    return crc;
}
static inline bool motor_hex(const char *text, size_t n, uint32_t *value)
{
    *value = 0;
    for (size_t i = 0; i < n; ++i) {
        unsigned digit;
        if (text[i] >= '0' && text[i] <= '9') digit = (unsigned)(text[i] - '0');
        else if (text[i] >= 'A' && text[i] <= 'F') digit = (unsigned)(text[i] - 'A') + 10;
        else return false;
        *value = (*value << 4) | digit;
    }
    return true;
}
static inline bool motor_number(const char *text, size_t n, uint32_t limit,
                                bool allow_negative, int32_t *value)
{
    bool negative = n && text[0] == '-';
    if (negative) {
        if (!allow_negative) return false;
        ++text; --n;
    }
    if (!n) return false;
    uint32_t magnitude = 0;
    for (size_t i = 0; i < n; ++i) {
        if (text[i] < '0' || text[i] > '9') return false;
        magnitude = magnitude * 10 + (unsigned)(text[i] - '0');
        if (magnitude > limit) return false;
    }
    *value = negative ? -(int32_t)magnitude : (int32_t)magnitude;
    return true;
}
static inline bool motor_parse(MotorParser *p)
{
    if (p->used < 18 || memcmp(p->data, "M,", 2) || p->data[10] != ',') return false;
    char *comma = strrchr(p->data, ',');
    uint32_t crc;
    if (!comma || strlen(comma + 1) != 4 || !motor_hex(comma + 1, 4, &crc) ||
        motor_crc(p->data, (size_t)(comma - p->data)) != crc ||
        !motor_hex(p->data + 2, 8, &p->sequence)) return false;
    *comma = '\0';
    const char *name = p->data + 11;
    p->left_pwm = p->right_pwm = 0;
    p->duration_ticks = 0;
    p->distance_mm = 0;
    p->reverse_move = false;
    p->angle_deg = 0;
    p->left_speed = p->right_speed = 0;
    p->arc_left_mm = p->arc_right_mm = 0;
    if (!strcmp(name, "STATUS")) p->command = CMD_STATUS;
    else if (!strcmp(name, "START")) p->command = CMD_START;
    else if (!strcmp(name, "STOP")) p->command = CMD_STOP;
    else if (!strcmp(name, "HB")) p->command = CMD_HEARTBEAT;
    else if (!strncmp(name, "DRIVE,", 6)) {
        const char *left = name + 6, *first = strchr(left, ',');
        if (!first) return false;
        const char *right = first + 1, *second = strchr(right, ',');
        if (!second) return false;
        const char *duration = second + 1;
        int32_t l, r, ms;
        if (!motor_number(left, (size_t)(first-left), MOTOR_PWM_LIMIT, true, &l) ||
            !motor_number(right, (size_t)(second-right), MOTOR_PWM_LIMIT, true, &r) ||
            !motor_number(duration, strlen(duration), 5000, false, &ms) ||
            ms < 10 || ms % 10) return false;
        p->command = CMD_DRIVE;
        p->left_pwm = (int16_t)l; p->right_pwm = (int16_t)r;
        p->duration_ticks = (uint16_t)(ms / 10);
    }
    else if (!strncmp(name, "MOVE,", 5)) {
        int32_t mm;
        if (!motor_number(name + 5, strlen(name + 5), 65535, true, &mm) || !mm) return false;
        p->command = CMD_MOVE;
        p->reverse_move = mm < 0;
        p->distance_mm = (uint16_t)(mm < 0 ? -mm : mm);
    }
    else if (!strncmp(name, "TURN,", 5)) {
        int32_t degrees;
        if (!motor_number(name + 5, strlen(name + 5), 30, true, &degrees) || !degrees) return false;
        p->command = CMD_TURN;
        p->angle_deg = (int16_t)degrees;
    }
    else if (!strncmp(name, "ARC,", 4)) {
        const char *left = name + 4, *split = strchr(left, ',');
        int32_t l, r;
        if (!split || !motor_number(left, (size_t)(split-left), 32767, true, &l) ||
            !motor_number(split+1, strlen(split+1), 32767, true, &r)) return false;
        p->command = CMD_ARC;
        p->arc_left_mm = (int16_t)l; p->arc_right_mm = (int16_t)r;
    }
    else if (!strncmp(name, "FOLLOW,", 7) || !strncmp(name, "SPEED,", 6)) {
        bool follow = name[0] == 'F';
        const char *left = name + (follow ? 7 : 6), *comma = strchr(left, ',');
        int32_t l, r;
        if (!comma || !motor_number(left, (size_t)(comma-left), 80, false, &l) ||
            !motor_number(comma+1, strlen(comma+1), 80, false, &r)) return false;
        p->command = follow ? CMD_FOLLOW : CMD_SPEED;
        p->left_speed = (uint16_t)l; p->right_speed = (uint16_t)r;
    }
    else return false;
    return true;
}
static inline bool motor_feed(MotorParser *p, uint8_t byte)
{
    if (byte == '@') { p->used = 0; p->active = true; return false; }
    if (!p->active) return false;
    if (byte == '\n') {
        p->active = false;
        p->data[p->used] = '\0';
        return motor_parse(p);
    }
    if (byte < 32 || byte > 126 || p->used >= sizeof(p->data) - 1) {
        p->active = false;
        return false;
    }
    p->data[p->used++] = (char)byte;
    return false;
}
#endif
