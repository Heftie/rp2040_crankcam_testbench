// Crank/cam testbench firmware. One binary, boot-time menu over USB
// serial (no reflashing needed to switch), two steps:
//   a) pick a crank/cam trigger-wheel profile (profiles.h)
//   b) pick a mode:
//      1) continuous double-buffered generation, RPM profile   (normal operation)
//      2) single-shot 2-rev diagnostic, constant RPM           (scope bring-up)
//      3) one-shot gen + 6ch capture + angle + pass/fail        (acquisition test)
//
// A selected mode runs forever (matches how each was validated standalone
// on real hardware) -- to pick a different profile or mode, reset/replug
// the board.

#include <stdio.h>
#include <assert.h>

#include "pico/stdlib.h"

#include "profiles.h"
#include "event_table.h"
#include "mode_continuous.h"
#include "mode_singleshot.h"
#include "mode_capture.h"

static const CrankCamProfile *select_profile_menu(void) {
    assert(crankcam_profile_count <= 9); // menu below assumes a single '1'..'9' digit
    printf("\ncrankcam testbench\n");
    printf("select crank/cam profile:\n");
    for (uint i = 0; i < crankcam_profile_count; i++) {
        printf(" %u) %s\n", i + 1, crankcam_profiles[i].name);
    }
    printf("profile: ");
    fflush(stdout);

    int c;
    do {
        c = getchar();
    } while (c < '1' || c > '0' + (int)crankcam_profile_count);
    printf("%c\n", (char)c);

    return &crankcam_profiles[c - '1'];
}

static int select_mode_menu(void) {
    printf(" 1) continuous double-buffered generation (normal operation)\n");
    printf(" 2) single-shot 2-rev diagnostic, constant RPM (scope bring-up)\n");
    printf(" 3) capture test: one-shot gen + 6ch capture + angle + pass/fail\n");
    printf("select mode (reset/replug to change profile or mode later): ");
    fflush(stdout);

    int c;
    do {
        c = getchar();
    } while (c != '1' && c != '2' && c != '3');
    printf("%c\n", (char)c);
    return c;
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500); // give the host time to open the serial port

    const CrankCamProfile *profile = select_profile_menu();
    select_profile(profile);

    int mode = select_mode_menu();

    if (mode == '1') {
        run_mode_continuous();
    } else if (mode == '2') {
        run_mode_singleshot();
    } else {
        run_mode_capture_test();
    }
    // unreachable: each mode loops forever
}
