// Crank/cam testbench firmware. One binary, three modes selected from a
// boot-time menu over USB serial (no reflashing needed to switch):
//   1) continuous double-buffered generation, RPM profile   (normal operation)
//   2) single-shot 2-rev diagnostic, constant RPM           (scope bring-up)
//   3) one-shot gen + 6ch capture + angle + pass/fail        (acquisition test)
//
// A selected mode runs forever (matches how each was validated standalone
// on real hardware) -- to pick a different mode, reset/replug the board.

#include <stdio.h>

#include "pico/stdlib.h"

#include "mode_continuous.h"
#include "mode_singleshot.h"
#include "mode_capture.h"

int main(void) {
    stdio_init_all();
    sleep_ms(1500); // give the host time to open the serial port

    printf("\ncrankcam testbench\n");
    printf(" 1) continuous double-buffered generation (normal operation)\n");
    printf(" 2) single-shot 2-rev diagnostic, constant RPM (scope bring-up)\n");
    printf(" 3) capture test: one-shot gen + 6ch capture + angle + pass/fail\n");
    printf("select mode (reset/replug to change mode later): ");
    fflush(stdout);

    int c;
    do {
        c = getchar();
    } while (c != '1' && c != '2' && c != '3');
    printf("%c\n", (char)c);

    if (c == '1') {
        run_mode_continuous();
    } else if (c == '2') {
        run_mode_singleshot();
    } else {
        run_mode_capture_test();
    }
    // unreachable: each mode loops forever
}
