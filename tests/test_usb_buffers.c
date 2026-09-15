/* Standalone test of the actual device-side implementation, without USB/SDK:
 * cc -std=c11 -Wall -Wextra -Werror -DUSB_BUFFERS_TEST \
 *   -Itests/usb_buffer_mocks tests/test_usb_buffers.c -o /tmp/test_usb_buffers
 * /tmp/test_usb_buffers
 * The guard keeps this translation unit inert in the Unity wildcard build.
 */
#ifdef USB_BUFFERS_TEST
#define PICO_ON_DEVICE 1
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <class/cdc/cdc_device.h>
#include <hardware/sync.h>
#include <pico/stdio.h>
#include <pico/time.h>

static bool connected;
static bool irq_enabled;
static bool worker_irq;
static uint32_t irq_saves, irq_restores, barriers;
static uint64_t now_us;
static uint32_t clock_reads;
static void (*rx_callback)(void *);
static void *rx_context;
static uint32_t callback_registrations;

static uint8_t wire[32768];
static size_t wire_length;
static uint32_t tx_capacity;
static uint32_t tx_write_limit;
static uint32_t tx_connected_calls, tx_available_calls, tx_write_calls, tx_flush_calls;
static uint32_t service_write_budget;
static uint32_t tx_call_budget;

static uint8_t ingress[4096];
static uint32_t ingress_read, ingress_length;
static uint32_t rx_available_calls, rx_read_calls;
static uint32_t rx_read_limit;
static uint32_t callback_read_budget;

uint32_t save_and_disable_interrupts(void) {
    uint32_t previous = irq_enabled ? 0u : 1u;
    irq_enabled = false;
    irq_saves++;
    return previous;
}
void restore_interrupts(uint32_t state) {
    assert(!irq_enabled && state <= 1u);
    irq_enabled = state == 0;
    irq_restores++;
}
void __dmb(void) {
    barriers++;
}
absolute_time_t get_absolute_time(void) {
    assert(worker_irq && !irq_enabled); /* No TX polling of a deadline/time source. */
    clock_reads++;
    return (absolute_time_t){.us_since_boot = now_us};
}
void stdio_set_chars_available_callback(void (*callback)(void *), void *context) {
    assert(callback != NULL);
    rx_callback = callback;
    rx_context = context;
    callback_registrations++;
}
static void check_tx_call(void) {
    assert(!irq_enabled && !worker_irq && tx_call_budget > 0);
    tx_call_budget--; /* Fail immediately on a busy loop, rather than hanging the test. */
}
bool tud_cdc_connected(void) {
    check_tx_call();
    tx_connected_calls++;
    return connected;
}
uint32_t tud_cdc_write_available(void) {
    check_tx_call();
    assert(connected);
    tx_available_calls++;
    return tx_capacity;
}
uint32_t tud_cdc_write(const void *buffer, uint32_t length) {
    check_tx_call();
    assert(connected);
    assert(buffer != NULL && length > 0 && length <= 64 && length <= tx_capacity);
    assert(service_write_budget > 0); /* Retrying a short/zero write would fail here. */
    service_write_budget--;
    tx_write_calls++;
    uint32_t count = length < tx_write_limit ? length : tx_write_limit;
    assert(wire_length + count <= sizeof(wire));
    memcpy(wire + wire_length, buffer, count);
    wire_length += count;
    tx_capacity -= count;
    return count;
}
uint32_t tud_cdc_write_flush(void) {
    check_tx_call();
    assert(connected);
    tx_flush_calls++;
    return 0; /* A blocked endpoint is not an invitation to wait/retry. */
}
uint32_t tud_cdc_available(void) {
    assert(worker_irq && !irq_enabled);
    rx_available_calls++;
    /* Also catches an unbounded loop that retries a zero-length read. */
    assert(rx_available_calls <= 9);
    return ingress_length - ingress_read;
}
uint32_t tud_cdc_read(void *buffer, uint32_t length) {
    assert(worker_irq && !irq_enabled);
    assert(buffer != NULL && length > 0 && length <= 64);
    assert(callback_read_budget > 0);
    callback_read_budget--;
    rx_read_calls++;
    uint32_t count = ingress_length - ingress_read;
    if (count > length) {
        count = length;
    }
    if (count > rx_read_limit) {
        count = rx_read_limit;
    }
    memcpy(buffer, ingress + ingress_read, count);
    ingress_read += count;
    return count;
}

/* Inclusion exposes only this test's static state for fixture reset/counter
 * wrap injection. Every enqueue, service, IRQ receive, and dequeue is real.
 */
#include "../src/usb_rx.c"
#include "../src/usb_tx.c"

static void reset_fixture(void) {
    memset(&urgent, 0, sizeof(urgent));
    memset(&ordinary, 0, sizeof(ordinary));
    memset(&active, 0, sizeof(active));
    active_offset = 0;
    dropped = 0;
    memset(chunks, 0, sizeof(chunks));
    read_index = write_index = 0;
    read_offset = 0;
    overflow = false;
    connected = true;
    irq_enabled = true;
    worker_irq = false;
    irq_saves = irq_restores = barriers = 0;
    now_us = 1000;
    clock_reads = 0;
    rx_callback = NULL;
    rx_context = NULL;
    callback_registrations = 0;
    wire_length = 0;
    tx_capacity = 0;
    tx_write_limit = UINT32_MAX;
    tx_connected_calls = tx_available_calls = tx_write_calls = tx_flush_calls = 0;
    service_write_budget = 0;
    tx_call_budget = 0;
    ingress_read = ingress_length = 0;
    rx_available_calls = rx_read_calls = 0;
    rx_read_limit = UINT32_MAX;
    callback_read_budget = 0;
    usb_rx_init();
    assert(callback_registrations == 1 && rx_callback != NULL && rx_context == NULL);
}

static void service_once(void) {
    const uint32_t saves = irq_saves;
    const uint32_t restores = irq_restores;
    const uint32_t connected_before = tx_connected_calls;
    const uint32_t available_before = tx_available_calls;
    const uint32_t writes_before = tx_write_calls;
    const uint32_t flushes_before = tx_flush_calls;
    const uint32_t time_before = clock_reads;
    const bool interrupts_before = irq_enabled;
    service_write_budget = 1;
    tx_call_budget = 4;
    usb_tx_service();
    assert(irq_saves == saves + 1 && irq_restores == restores + 1);
    assert(irq_enabled == interrupts_before);
    assert(tx_connected_calls == connected_before + 1);
    assert(tx_available_calls - available_before <= 1);
    assert(tx_write_calls - writes_before <= 1 && tx_flush_calls - flushes_before <= 1);
    assert(clock_reads == time_before); /* No timeout loop, sleeps, or USB task API provided. */
}

static bool enqueue(const uint8_t *data, size_t length, bool priority) {
    const uint32_t writes_before = tx_write_calls;
    const uint32_t saves = irq_saves;
    const uint32_t restores = irq_restores;
    service_write_budget = 2; /* At most one write in each of the two service calls. */
    tx_call_budget = 8;
    bool result = usb_tx_packet(data, length, priority);
    assert(irq_enabled && irq_saves - saves <= 2 && irq_restores - restores <= 2);
    assert(irq_saves == irq_restores);
    assert(tx_write_calls - writes_before <= 2);
    return result;
}

static void drain_tx(void) {
    tx_capacity = sizeof(wire) - (uint32_t)wire_length;
    for (unsigned budget = 0; budget < 4096; ++budget) {
        service_once();
        if (active.length == 0 && urgent.read == urgent.write && ordinary.read == ordinary.write) {
            return;
        }
    }
    assert(!"TX failed to drain within its finite service budget");
}

static void expect_wire(const uint8_t *expected, size_t length) {
    assert(wire_length == length);
    assert(memcmp(wire, expected, length) == 0);
}

static void feed_rx(const uint8_t *bytes, size_t length) {
    assert(ingress_read == ingress_length && length <= sizeof(ingress));
    memcpy(ingress, bytes, length);
    ingress_read = 0;
    ingress_length = (uint32_t)length;
}

static void receive_irq(void) {
    assert(irq_enabled && !worker_irq && rx_callback != NULL);
    const uint32_t time_before = clock_reads;
    irq_enabled = false;
    worker_irq = true;
    callback_read_budget = 8;
    rx_available_calls = rx_read_calls = 0;
    rx_callback(rx_context);
    assert(!irq_enabled && rx_read_calls <= 8 && clock_reads == time_before + 1);
    worker_irq = false;
    irq_enabled = true;
}

static void expect_rx(const uint8_t *expected, size_t length, uint64_t arrival_us) {
    for (size_t i = 0; i < length; ++i) {
        absolute_time_t arrival = {.us_since_boot = UINT64_MAX};
        assert(usb_rx_get(&arrival) == expected[i]);
        assert(arrival.us_since_boot == arrival_us);
    }
}

static void expect_rx_empty(void) {
    absolute_time_t untouched = {.us_since_boot = 123};
    assert(usb_rx_get(&untouched) == PICO_ERROR_TIMEOUT);
    assert(untouched.us_since_boot == 123);
}

static void test_blocked_and_short_tx(void) {
    reset_fixture();
    uint8_t packet[200];
    for (size_t i = 0; i < sizeof(packet); ++i) {
        packet[i] = (uint8_t)i;
    }
    assert(enqueue(packet, sizeof(packet), false));
    for (unsigned i = 0; i < 100; ++i) {
        service_once();
    }
    assert(wire_length == 0 && tx_write_calls == 0 && tx_flush_calls == 0);
    assert(active_offset == 0 && active.length == sizeof(packet));
    tx_capacity = 1000;
    tx_write_limit = 0; /* Availability can race with an unexpectedly zero write. */
    for (unsigned i = 0; i < 10; ++i) {
        service_once();
    }
    assert(wire_length == 0 && active_offset == 0 && tx_write_calls == 10);
    tx_write_limit = 3; /* Offset must advance by accepted bytes, not requested bytes. */
    drain_tx();
    expect_wire(packet, sizeof(packet));
    assert(tx_flush_calls == tx_write_calls && usb_tx_dropped() == 0);
    irq_enabled = false;
    service_once(); /* Restore the caller's masked state, not always "enable". */
    assert(!irq_enabled);
    irq_enabled = true;
}

static void test_priority_without_interleaving(void) {
    reset_fixture();
    uint8_t first[150], later[19], urgent_first[13], urgent_second[9], expected[191];
    memset(first, 'A', sizeof(first));
    memset(later, 'B', sizeof(later));
    memset(urgent_first, 'C', sizeof(urgent_first));
    memset(urgent_second, 'D', sizeof(urgent_second));
    tx_capacity = 7;
    assert(enqueue(first, sizeof(first), false));
    assert(wire_length == 7 && active_offset == 7);
    assert(enqueue(later, sizeof(later), false));
    assert(enqueue(urgent_first, sizeof(urgent_first), true));
    assert(enqueue(urgent_second, sizeof(urgent_second), true));
    memcpy(expected, first, sizeof(first));
    memcpy(expected + 150, urgent_first, sizeof(urgent_first));
    memcpy(expected + 163, urgent_second, sizeof(urgent_second));
    memcpy(expected + 172, later, sizeof(later));
    /* Queuing must copy the packet; application buffers can be reused immediately. */
    memset(first, 'x', sizeof(first));
    memset(later, 'x', sizeof(later));
    memset(urgent_first, 'x', sizeof(urgent_first));
    memset(urgent_second, 'x', sizeof(urgent_second));
    tx_write_limit = 11;
    drain_tx();
    expect_wire(expected, sizeof(expected));
}

static void test_tx_capacity_and_wrap(void) {
    reset_fixture();
    urgent.read = urgent.write = UINT32_MAX - 3u;
    ordinary.read = ordinary.write = UINT32_MAX - 3u;
    uint8_t packet[786];
    uint8_t expected[17 * 786];
    memset(packet, 0x10, sizeof(packet));
    assert(enqueue(packet, sizeof(packet), false)); /* One active packet, not a queue slot. */
    memcpy(expected, packet, sizeof(packet));
    for (size_t i = 0; i < 8; ++i) {
        memset(packet, (int)(0x20 + i), sizeof(packet));
        assert(enqueue(packet, sizeof(packet), false));
        memcpy(expected + (9 + i) * sizeof(packet), packet, sizeof(packet));
        memset(packet, (int)(0x40 + i), sizeof(packet));
        assert(enqueue(packet, sizeof(packet), true));
        memcpy(expected + (1 + i) * sizeof(packet), packet, sizeof(packet));
    }
    assert(ordinary.write - ordinary.read == 8 && urgent.write - urgent.read == 8);
    assert(!enqueue(packet, sizeof(packet), false));
    assert(!enqueue(packet, sizeof(packet), true));
    assert(!enqueue(packet, 787, true)); /* Length rejected before touching the data. */
    assert(!enqueue(packet, 0, false));
    assert(usb_tx_dropped() == 4);
    drain_tx();
    expect_wire(expected, sizeof(expected));
    assert(enqueue(packet, 1, false)); /* Capacity recovered after wrap and drain. */
    drain_tx();
    assert(wire_length == sizeof(expected) + 1 && wire[sizeof(expected)] == packet[0]);
}

static void test_discard_and_disconnect(void) {
    reset_fixture();
    const uint8_t first[] = {1, 2, 3, 4, 5};
    const uint8_t telemetry[] = {6, 7, 8};
    const uint8_t reply[] = {9, 10};
    tx_capacity = 2;
    assert(enqueue(first, sizeof(first), false));
    assert(enqueue(telemetry, sizeof(telemetry), false));
    assert(enqueue(reply, sizeof(reply), true));
    usb_tx_discard_telemetry();
    drain_tx();
    const uint8_t expected[] = {1, 2, 3, 4, 5, 9, 10};
    expect_wire(expected, sizeof(expected)); /* Never truncate the active frame. */

    reset_fixture();
    tx_capacity = 2;
    assert(enqueue(first, sizeof(first), false));
    assert(enqueue(telemetry, sizeof(telemetry), false));
    assert(enqueue(reply, sizeof(reply), true));
    connected = false;
    const uint32_t writes = tx_write_calls;
    service_once();
    assert(active.length == 0 && active_offset == 0);
    assert(ordinary.read == ordinary.write && urgent.read == urgent.write);
    assert(tx_write_calls == writes);
    (void)enqueue(first, sizeof(first), true); /* Offline enqueues cannot leak after reconnect. */
    assert(urgent.read == urgent.write && ordinary.read == ordinary.write);
    wire_length = 0; /* New host connection has a separate byte stream. */
    connected = true;
    tx_capacity = 100;
    service_once();
    assert(wire_length == 0);
    assert(enqueue(reply, sizeof(reply), true));
    drain_tx();
    expect_wire(reply, sizeof(reply));
}

static void test_irq_timestamp_and_budget(void) {
    reset_fixture();
    uint8_t bytes[900];
    for (size_t i = 0; i < sizeof(bytes); ++i) {
        bytes[i] = (uint8_t)i;
    }
    feed_rx(bytes, sizeof(bytes));
    now_us = 10000;
    receive_irq();
    assert(rx_read_calls == 8 && ingress_read == 512 && write_index == 8);
    now_us = 20000;
    receive_irq();
    assert(rx_read_calls == 7 && ingress_read == sizeof(bytes));
    now_us = 2000000; /* Main loop stalled well beyond the control lease. */
    const uint32_t time_before = clock_reads;
    expect_rx(bytes, 13, 10000);
    now_us += 1000000;
    expect_rx(bytes + 13, 499, 10000);
    expect_rx(bytes + 512, sizeof(bytes) - 512, 20000);
    assert(clock_reads == time_before); /* Dequeue cannot renew an ingress timestamp. */
    expect_rx_empty();
    assert(!usb_rx_take_overflow() && irq_enabled && barriers > 0);
}

static void test_rx_capacity_overflow_and_recovery(void) {
    reset_fixture();
    read_index = write_index = UINT32_MAX - 15u;
    uint8_t bytes[2048];
    for (size_t i = 0; i < sizeof(bytes); ++i) {
        bytes[i] = (uint8_t)(i * 13u);
    }
    feed_rx(bytes, sizeof(bytes));
    for (unsigned irq = 0; irq < 4; ++irq) {
        receive_irq();
    }
    assert(write_index - read_index == 32 && ingress_read == sizeof(bytes));
    assert(!usb_rx_take_overflow()); /* All 32 slots, not 31, are usable. */
    expect_rx(bytes, sizeof(bytes), now_us);
    expect_rx_empty();

    feed_rx(bytes, sizeof(bytes));
    for (unsigned irq = 0; irq < 4; ++irq) {
        receive_irq();
    }
    expect_rx(bytes, 17, now_us); /* Full queue with a partially consumed head. */
    assert(read_offset == 17 && write_index - read_index == 32);
    feed_rx(bytes, 512);
    const uint32_t published = write_index;
    receive_irq();
    assert(rx_read_calls == 8 && ingress_read == 512 && write_index == published);
    assert(usb_rx_take_overflow());
    assert(read_index == write_index && read_offset == 0 && irq_enabled);
    expect_rx_empty(); /* No prefix, suffix, or following queued bytes survive the gap. */
    assert(!usb_rx_take_overflow()); /* Sticky event is consumed exactly once. */
    const uint8_t fresh[] = {0xf0, 0, 0xff, 17};
    feed_rx(fresh, sizeof(fresh));
    now_us += 123456;
    receive_irq();
    expect_rx(fresh, sizeof(fresh), now_us);
    expect_rx_empty();
    irq_enabled = false;
    assert(!usb_rx_take_overflow());
    assert(!irq_enabled);
    irq_enabled = true;
}

static void test_rx_zero_and_short_reads(void) {
    reset_fixture();
    receive_irq();
    assert(rx_read_calls == 0 && write_index == 0);
    expect_rx_empty();
    const uint8_t bytes[] = {0, 1, 127, 128, 254, 255, 42};
    feed_rx(bytes, sizeof(bytes));
    rx_read_limit = 0;
    receive_irq();
    assert(rx_read_calls == 1 && write_index == 0 && ingress_read == 0);
    expect_rx_empty();
    rx_read_limit = 2;
    now_us = 456;
    receive_irq();
    assert(rx_read_calls == 4 && write_index == 4);
    expect_rx(bytes, sizeof(bytes), 456);
    expect_rx_empty();

    /* Short hardware reads must still consume slots, never overrun storage. */
    uint8_t more[40];
    memset(more, 0xaa, sizeof(more));
    feed_rx(more, sizeof(more));
    rx_read_limit = 1;
    for (unsigned irq = 0; irq < 4; ++irq) {
        receive_irq();
    }
    assert(write_index - read_index == 32 && ingress_read == 32);
    rx_read_limit = 0; /* Even a full ring plus a zero discard read is bounded. */
    receive_irq();
    assert(rx_read_calls == 8 && ingress_read == 32);
    assert(usb_rx_take_overflow());
    expect_rx_empty();
    rx_read_limit = UINT32_MAX;
    receive_irq();
    expect_rx(more + 32, 8, now_us);
    expect_rx_empty();
}

int main(void) {
    test_blocked_and_short_tx();
    test_priority_without_interleaving();
    test_tx_capacity_and_wrap();
    test_discard_and_disconnect();
    test_irq_timestamp_and_budget();
    test_rx_capacity_overflow_and_recovery();
    test_rx_zero_and_short_reads();
    puts("USB buffers: bounded TX, frame ordering, disconnect, IRQ timestamps, RX overflow and "
         "wrap passed");
    return 0;
}
#endif
