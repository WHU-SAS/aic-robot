#include "uart_request_parser.h"
#include <assert.h>
#include <stdio.h>

static unsigned feed(UartRequestParser *s, const void *data, size_t size)
{
    const uint8_t *p = data;
    unsigned received = 0;
    for (size_t i = 0; i < size; ++i) received += uart_request_feed(s, p[i]);
    return received;
}
#define TEXT(s, text) feed(s, text, sizeof(text)-1)
int main(void)
{
    UartRequestParser s = {0};
    assert(TEXT(&s, "K230_TEST,01,1234ABCD\n") == 1);
    assert(strcmp(s.line, "K230_TEST,01,1234ABCD") == 0);
    const uint8_t startup[] = {0, 255, 0, '\n', 'X', 'X'};
    assert(feed(&s, startup, sizeof(startup)) == 0);
    assert(TEXT(&s, "K230_TEST,01,00000000\n") == 1);
    assert(TEXT(&s, "K230_TEST,01,BADK230_TEST,02,DEADBEEF\n") == 1);
    assert(strcmp(s.line, "K230_TEST,02,DEADBEEF") == 0);
    assert(TEXT(&s, "KK230_TEST,03,12345678\r\n") == 1);
    assert(TEXT(&s, "K230_TEST,04,1234Z678\n") == 0);
    assert(TEXT(&s, "K230_TEST,XX,12345678\n") == 0);
    assert(TEXT(&s, "K230_TEST,05,123456789\n") == 0);
    assert(TEXT(&s, "K230_TEST,05,1234567\n") == 0);
    assert(TEXT(&s, "K230_TEST,06,12345678\nK230_TEST,07,12345678\n") == 2);
    puts("PASS: clean request, startup noise, header resync, strict token, CRLF, back-to-back");
}
