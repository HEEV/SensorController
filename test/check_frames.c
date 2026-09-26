/*
 * Check that what the firmware actually transmitted is what the Raspberry Pi
 * can actually decode.
 *
 * Input is raw UART bytes captured from the firmware running on a simulated
 * ATmega328p. They are fed through SensorHub's real parser, the same code the
 * car runs. Both halves of the wire format are therefore checked against each
 * other with no hardware in the loop.
 *
 *     check_frames <uart.bin> <minimum packets>
 *
 * Fails on anything less than a clean run: a CRC error, an unrecognised
 * format, a resync, a gap in the sequence numbers, or too few packets.
 */

#include <sensorhub/sensorhub.h>

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    FILE *f;
    sh_parser_t parser;
    sh_packet_t packet;
    long accepted = 0;
    long want;
    int c;
    int failures = 0;
    uint16_t first = 0, last = 0;

    if (argc < 3) {
        fprintf(stderr, "usage: check_frames <uart.bin> <minimum packets>\n");
        return 2;
    }

    want = atol(argv[2]);

    f = fopen(argv[1], "rb");
    if (f == NULL) {
        perror(argv[1]);
        return 2;
    }

    sh_parser_init(&parser);

    while ((c = fgetc(f)) != EOF) {
        if (sh_parser_feed(&parser, (uint8_t)c, &packet) == SH_OK) {
            if (accepted == 0) {
                first = packet.sequence;
                printf("first packet: seq=%u speed=%.2f air=%.2f "
                       "eng=%.1f rad=%.1f A0=%u in=0x%02X out=0x%02X\n",
                       packet.sequence, (double)packet.speed,
                       (double)packet.airspeed, (double)packet.temps[0],
                       (double)packet.temps[1], packet.analog[0],
                       packet.digital_in, packet.digital_out);
            }
            last = packet.sequence;
            accepted++;
        }
    }

    fclose(f);

    printf("\naccepted       %ld\n", accepted);
    printf("sequence       %u .. %u\n", first, last);
    printf("crc errors     %llu\n", (unsigned long long)parser.stats.checksum_errors);
    printf("format errors  %llu\n", (unsigned long long)parser.stats.format_errors);
    printf("resyncs        %llu\n", (unsigned long long)parser.stats.resyncs);
    printf("dropped        %llu\n\n", (unsigned long long)parser.stats.dropped);

    if (accepted < want) {
        printf("FAIL: wanted at least %ld packets\n", want);
        failures++;
    }
    if (parser.stats.checksum_errors != 0) {
        printf("FAIL: the firmware emitted frames the parser could not verify\n");
        failures++;
    }
    if (parser.stats.format_errors != 0) {
        printf("FAIL: format byte disagreement between firmware and library\n");
        failures++;
    }
    if (parser.stats.resyncs != 0) {
        printf("FAIL: lost framing on a clean simulated link\n");
        failures++;
    }
    if (parser.stats.dropped != 0) {
        printf("FAIL: gap in the sequence numbers\n");
        failures++;
    }

    /* A clean run of N packets must span exactly N sequence numbers. */
    if (accepted > 0 && (uint16_t)(last - first) != (uint16_t)(accepted - 1)) {
        printf("FAIL: %ld packets but the sequence spans %u\n", accepted,
               (unsigned)(uint16_t)(last - first + 1));
        failures++;
    }

    if (failures == 0) {
        printf("PASS: the firmware and the library agree on the wire\n");
    }

    return failures == 0 ? 0 : 1;
}
