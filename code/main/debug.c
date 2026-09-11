#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "debug.h"

#if DEBUG_ENABLED

#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

static volatile char     s_buf[DEBUG_TX_BUF];
static volatile uint8_t  s_head = 0;
static volatile uint8_t  s_tail = 0;
static volatile uint16_t s_dropped = 0;

void Debug_Init(void) {
    UBRRH = (uint8_t)(UBRR_VALUE >> 8);
    UBRRL = (uint8_t)UBRR_VALUE;
    UCSRB = (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);   // 8N1
    s_head = s_tail = 0;
    s_dropped = 0;
}

// Drains the queue one byte per interrupt. Disables itself when empty so the
// interrupt does not fire continuously with nothing to send.
ISR(USART_UDRE_vect) {
    if (s_head == s_tail) {
        UCSRB &= ~(1 << UDRIE);
        return;
    }
    UDR = s_buf[s_tail];
    s_tail = (uint8_t)((s_tail + 1) % DEBUG_TX_BUF);
}

static void tx_push(char c) {
    uint8_t next;

    // Before sei(), or inside a critical section, the ISR cannot run -- fall
    // back to a blocking send so start-up messages are not silently lost.
    if (!(SREG & (1 << SREG_I))) {
        while (!(UCSRA & (1 << UDRE)));
        UDR = c;
        return;
    }

    next = (uint8_t)((s_head + 1) % DEBUG_TX_BUF);
    if (next == s_tail) {
        // Buffer full. DROP the byte. Never spin here: blocking would push
        // the control loop past its tick deadline, which is exactly the
        // failure mode this module exists to avoid.
        if (s_dropped < 0xFFFF) s_dropped++;
        return;
    }
    s_buf[s_head] = c;
    s_head = next;
    UCSRB |= (1 << UDRIE);
}

void Debug_Str(const char *s) {
    while (*s) tx_push(*s++);
}

void Debug_Int(int32_t v) {
    char buf[12];
    char *p = buf;
    ltoa(v, buf, 10);
    while (*p) tx_push(*p++);
}

void Debug_NL(void) { tx_push('\r'); tx_push('\n'); }

void Debug_KV(const char *key, int32_t v) {
    Debug_Str(key);
    tx_push('=');
    Debug_Int(v);
    tx_push(' ');
}

void Debug_CSV(int32_t v) {
    Debug_Int(v);
    tx_push(',');
}

uint16_t Debug_Dropped(void) { return s_dropped; }

void Debug_Flush(void) {
    while (s_head != s_tail) { /* let the ISR drain it */ }
    while (!(UCSRA & (1 << UDRE)));
}

#else
void Debug_Init(void) {}
void Debug_Str(const char *s) { (void)s; }
void Debug_Int(int32_t v) { (void)v; }
void Debug_NL(void) {}
void Debug_KV(const char *key, int32_t v) { (void)key; (void)v; }
void Debug_CSV(int32_t v) { (void)v; }
uint16_t Debug_Dropped(void) { return 0; }
void Debug_Flush(void) {}
#endif
