#ifndef UART_REQUEST_PARSER_H
#define UART_REQUEST_PARSER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define UART_REQUEST_PREFIX "K230_TEST,"
#define UART_REQUEST_PREFIX_LEN 10U
#define UART_REQUEST_LINE_LEN 21U

typedef struct {
    char line[UART_REQUEST_LINE_LEN + 1U];
    size_t used;
    size_t matched;
    bool receiving;
} UartRequestParser;

static inline bool uart_request_hex(char c)
{
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
           (c >= 'a' && c <= 'f');
}

/* Resynchronize at a complete header, including after noise or a partial frame.
 * A request is exactly K230_TEST,<two decimal digits>,<eight hex digits>\n.
 */
static inline bool uart_request_feed(UartRequestParser *s, uint8_t byte)
{
    if (byte == (uint8_t)UART_REQUEST_PREFIX[s->matched]) {
        ++s->matched;
    } else {
        s->matched = byte == (uint8_t)UART_REQUEST_PREFIX[0] ? 1U : 0U;
    }
    if (s->matched == UART_REQUEST_PREFIX_LEN) {
        memcpy(s->line, UART_REQUEST_PREFIX, UART_REQUEST_PREFIX_LEN);
        s->used = UART_REQUEST_PREFIX_LEN;
        s->matched = 0U;
        s->receiving = true;
        return false;
    }
    if (!s->receiving || byte == '\r') return false;
    if (byte == '\n') {
        s->receiving = false;
        if (s->used != UART_REQUEST_LINE_LEN || s->line[12] != ',') return false;
        if (s->line[10] < '0' || s->line[10] > '9' ||
            s->line[11] < '0' || s->line[11] > '9') return false;
        for (size_t i = 13U; i < UART_REQUEST_LINE_LEN; ++i) {
            if (!uart_request_hex(s->line[i])) return false;
        }
        s->line[s->used] = '\0';
        return true;
    }
    if (byte < 0x20U || byte > 0x7EU || s->used >= UART_REQUEST_LINE_LEN) {
        s->receiving = false;
        return false;
    }
    s->line[s->used++] = (char)byte;
    return false;
}
#endif
