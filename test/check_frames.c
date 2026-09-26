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
#include <stdbool.h>

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

    {
        /* A table, not six copies of one branch. The seventh check is data. */
        const struct { bool bad; const char *why; } checks[] = {
            { accepted < want,
              "too few packets" },
            { parser.stats.checksum_errors != 0,
              "frames the parser could not verify" },
            { parser.stats.format_errors != 0,
              "format byte disagreement between firmware and library" },
            { parser.stats.resyncs != 0,
              "lost framing on a clean simulated link" },
            { parser.stats.dropped != 0,
              "gap in the sequence numbers" },
            /* a clean run of N packets spans exactly N sequence numbers */
            { accepted > 0 &&
                  (uint16_t)(last - first) != (uint16_t)(accepted - 1),
              "packet count and sequence span disagree" },
        };

        for (size_t i = 0; i < sizeof(checks) / sizeof(checks[0]); ++i) {
            if (checks[i].bad) {
                printf("FAIL: %s\n", checks[i].why);
                failures++;
            }
        }
    }

    if (failures == 0) {
        printf("PASS: the firmware and the library agree on the wire\n");
    }

    return failures == 0 ? 0 : 1;
}
