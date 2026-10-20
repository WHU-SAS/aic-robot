#ifndef AIC_MOTOR_PROTOCOL_H
#define AIC_MOTOR_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* CRC-16/CCITT-FALSE: init FFFF, poly 1021, no reflection/xorout.
 * Request: @M,01234567,START,ABCD\n. CRC covers M,...,START only.
 * An '@' always resynchronizes, including inside an incomplete frame. */
typedef enum { CMD_STATUS, CMD_START, CMD_STOP, CMD_HEARTBEAT } MotorCommand;
typedef struct {
    char data[48];
    size_t used;
    bool active;
    uint32_t sequence;
    MotorCommand command;
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
    if (!strcmp(name, "STATUS")) p->command = CMD_STATUS;
    else if (!strcmp(name, "START")) p->command = CMD_START;
    else if (!strcmp(name, "STOP")) p->command = CMD_STOP;
    else if (!strcmp(name, "HB")) p->command = CMD_HEARTBEAT;
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
