// Crank/cam testbench firmware. Command-driven over USB serial, no
// boot-time menu and no reset needed to change anything: generation and
// capture are independently entered/left at any time, RPM is settable
// on the fly, and the trigger-wheel profile is switchable while
// generation is stopped. See engine.h for the state machine this drives.
//
// Commands are one line each (LF or CRLF terminated), a letter followed
// by an optional argument, no separator:
//   p<n>   select trigger-wheel profile n (see 'l' for the list); gen
//          must be stopped
//   r<n>   set RPM to n; applies at the next 720deg cycle boundary if
//          generation is already running
//   g1/g0  start/stop crank+cam generation
//   c1/c0  start/stop capture + live per-cycle angle report; requires
//          generation to be running
//   l      list trigger-wheel profiles
//   ?      print current status
//   h      print this help
//
// Every command prints exactly one "OK ..." or "ERR ..." response line,
// except capture reports, which print as their own "cycle N:" blocks
// whenever a capture buffer completes (independent of command input).

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico/stdlib.h"

#include "engine.h"
#include "profiles.h"

static void print_help(void) {
    printf("commands:\n");
    printf("  p<n>   select trigger-wheel profile n (see 'l'); requires gen=0\n");
    printf("  r<n>   set RPM to n (1-%u); live if gen=1\n", ENGINE_MAX_RPM);
    printf("  g1/g0  start/stop generation\n");
    printf("  c1/c0  start/stop capture + live angle report; requires gen=1\n");
    printf("  l      list trigger-wheel profiles\n");
    printf("  ?      print status\n");
    printf("  h      print this help\n");
}

static void print_profiles(void) {
    for (uint i = 0; i < crankcam_profile_count; i++) {
        printf("  %u) %s\n", i + 1, crankcam_profiles[i].name);
    }
}

static void print_status(void) {
    printf("STATUS profile=%s rpm=%lu gen=%d capture=%d\n",
           engine_profile()->name, (unsigned long)engine_rpm(),
           engine_gen_running() ? 1 : 0, engine_capture_running() ? 1 : 0);
}

static void handle_command(const char *line) {
    if (line[0] == '\0') return;

    char cmd = line[0];
    const char *arg = line + 1;

    switch (cmd) {
    case 'h':
        print_help();
        printf("OK\n");
        break;

    case 'l':
        print_profiles();
        printf("OK\n");
        break;

    case '?':
        print_status();
        break;

    case 'p': {
        char *end;
        long n = strtol(arg, &end, 10);
        if (end == arg || n < 1 || n > (long)crankcam_profile_count) {
            printf("ERR profile must be 1-%u\n", crankcam_profile_count);
        } else if (!engine_select_profile((uint)n)) {
            printf("ERR stop generation (g0) before changing profile\n");
        } else {
            printf("OK profile=%s\n", engine_profile()->name);
        }
        break;
    }

    case 'r': {
        char *end;
        long n = strtol(arg, &end, 10);
        if (end == arg || n < 1 || n > (long)ENGINE_MAX_RPM) {
            printf("ERR rpm must be 1-%u\n", ENGINE_MAX_RPM);
        } else {
            engine_set_rpm((uint32_t)n);
            printf("OK rpm=%ld\n", n);
        }
        break;
    }

    case 'g':
        if (strcmp(arg, "1") == 0) {
            engine_start_gen();
            printf("OK gen=1\n");
        } else if (strcmp(arg, "0") == 0) {
            engine_stop_gen();
            printf("OK gen=0\n");
        } else {
            printf("ERR usage: g1 or g0\n");
        }
        break;

    case 'c':
        if (strcmp(arg, "1") == 0) {
            if (!engine_gen_running()) {
                printf("ERR start generation (g1) before capture\n");
            } else if (engine_rpm() < CAPTURE_MIN_RPM) {
                printf("ERR rpm too low for capture (min %u)\n", CAPTURE_MIN_RPM);
            } else {
                engine_start_capture();
                printf("OK capture=1\n");
            }
        } else if (strcmp(arg, "0") == 0) {
            engine_stop_capture();
            printf("OK capture=0\n");
        } else {
            printf("ERR usage: c1 or c0\n");
        }
        break;

    default:
        printf("ERR unknown command: %s\n", line);
        break;
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500); // give the host time to open the serial port

    engine_init();

    printf("crankcam testbench ready. 'h' for help.\n");
    print_status();

    char line[32];
    uint len = 0;

    while (true) {
        engine_poll_capture();

        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (len > 0) {
                line[len] = '\0';
                handle_command(line);
                len = 0;
            }
            continue;
        }
        if (len < sizeof(line) - 1) {
            line[len++] = (char)c;
        }
        // else: silently drop the rest of an overlong line
    }
}
