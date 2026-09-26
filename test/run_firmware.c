/*
 * Run the compiled firmware on a simulated ATmega328p and capture the raw
 * bytes it puts on the UART.
 *
 * Why not just read simavr's console output: that is a pretty-printer. It
 * substitutes '.' for every non-printable byte, so the format byte 0x01 comes
 * out as 0x2E and nothing parses. Hooking UART_IRQ_OUTPUT gives the bytes the
 * chip would actually have transmitted.
 *
 *     run_firmware <firmware.elf> <out.bin> <microseconds>
 *
 * The budget is in simulated microseconds, not iterations: avr_run() executes
 * one instruction, so counting calls measures something nobody cares about.
 */

#include <simavr/avr_uart.h>
#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/sim_irq.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MCU "atmega328p"
#define FREQ 16000000

static FILE *out;
static long captured;

static void on_uart_byte(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    fputc((int)(value & 0xFF), out);
    captured++;
}

int main(int argc, char **argv)
{
    elf_firmware_t fw;
    avr_t *avr;
    avr_irq_t *irq;
    avr_cycle_count_t limit;

    memset(&fw, 0, sizeof(fw));

    if (argc < 4) {
        fprintf(stderr, "usage: run_firmware <elf> <out.bin> <microseconds>\n");
        return 2;
    }

    if (elf_read_firmware(argv[1], &fw) != 0) {
        fprintf(stderr, "could not read %s\n", argv[1]);
        return 2;
    }

    avr = avr_make_mcu_by_name(MCU);
    if (avr == NULL) {
        fprintf(stderr, "simavr does not know " MCU "\n");
        return 2;
    }

    avr_init(avr);
    fw.frequency = FREQ;
    avr_load_firmware(avr, &fw);
    avr->frequency = FREQ;

    out = fopen(argv[2], "wb");
    if (out == NULL) {
        perror(argv[2]);
        return 2;
    }

    /* Console echo off: it is a pretty-printer that substitutes '.' for
       non-printable bytes, and we want the real ones. */
    {
        uint32_t flags = 0;
        avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &flags);
        flags &= (uint32_t)~AVR_UART_FLAG_STDIO;
        avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);
    }

    irq = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT);
    if (irq == NULL) {
        fprintf(stderr, "no UART output irq on " MCU "\n");
        return 2;
    }
    avr_irq_register_notify(irq, on_uart_byte, NULL);

    limit = (avr_cycle_count_t)((double)atoll(argv[3]) * (double)FREQ / 1e6);

    while (avr->cycle < limit) {
        int state = avr_run(avr);
        if (state == cpu_Done || state == cpu_Crashed) {
            fprintf(stderr, "firmware stopped early: state %d\n", state);
            break;
        }
    }

    fclose(out);

    /* Known limitation: capture stops after roughly 50 frames, whatever the
       budget, because the AVR's TX ring fills and simavr does not drain it.
       The firmware keeps running, wedged in HardwareSerial::write. So treat
       the frame count as a sample, not as a rate: it says nothing about how
       fast the loop runs, and comparing counts between builds measures where
       the simulator stalls rather than anything about the code. */
    fprintf(stderr, "captured %ld bytes, %.3f s simulated (stops early, see note)\n",
            captured, (double)avr->cycle / (double)FREQ);

    if (captured == 0) {
        fprintf(stderr, "the firmware transmitted nothing\n");
        return 1;
    }

    return 0;
}
