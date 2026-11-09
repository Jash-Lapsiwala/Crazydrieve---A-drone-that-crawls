import signal
import threading
import time
import traceback
from bisect import bisect_left
from bisect import bisect_right
from datetime import datetime

from openpyxl import Workbook

import cflib.crtp as crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = "radio://0/80/2M"

# Table-edge detection now runs in the firmware (localMin module). This is the
# range jump between two consecutive ToF readings that counts as an edge. Set it
# to about half the table height; it is sent to localMin.stepThresh each flight.
STEP_THRESH_M = 0.15
FORWARD_M = 2.0             # distance of the forward leg in meters

# True: the kill switch also sends the firmware's emergency stop, which locks the
# motors until the Crazyflie is power-cycled. False: it only sends a stop setpoint.
KILL_USES_EMERGENCY_STOP = True

# The Excel sheet gets one row per sample of this group; the other groups'
# latest values are carried forward onto each row.
CLOCK_GROUP = "Pose"

# Each group becomes its own log block and its own Excel sheet.
# A single log block can hold at most 26 bytes, so the variables are split up.
# (sheet name, period in ms, [(variable, type), ...])
LOG_GROUPS = [
    ("Pose", 20, [
        ("stateEstimate.x", "float"),
        ("stateEstimate.y", "float"),
        ("stateEstimate.z", "float"),
        ("stateEstimate.roll", "float"),
        ("stateEstimate.pitch", "float"),
        ("stateEstimate.yaw", "float"),
    ]),
    ("Battery", 500, [
        ("pm.vbat", "float"),
        ("pm.batteryLevel", "uint8_t"),
    ]),
    # Raw motor commands (0..65535), the final control effort sent to the motors
    ("Motors", 20, [
        ("motor.m1", "uint32_t"),
        ("motor.m2", "uint32_t"),
        ("motor.m3", "uint32_t"),
        ("motor.m4", "uint32_t"),
    ]),
    # Controller outputs before mixing into motor commands
    ("Controller", 20, [
        ("controller.cmd_thrust", "float"),
        ("controller.cmd_roll", "float"),
        ("controller.cmd_pitch", "float"),
        ("controller.cmd_yaw", "float"),
    ]),
    # Raw downward ToF range in mm. Best data for tuning STEP_THRESH_M.
    ("Zrange", 10, [
        ("range.zrange", "uint16_t"),
    ]),
    # Firmware table-edge module. Names must match the LOG_GROUP in the .c file.
    ("LocalMin", 20, [
        ("localMin.target", "float"),
        ("localMin.overTable", "uint8_t"),
        ("localMin.count", "uint8_t"),
    ]),
]


class FlightRecorder:
    """Collects log data in memory during flight and writes it to Excel afterwards."""

    def __init__(self):
        self.t0 = time.time()
        self.rows = {name: [] for name, _, _ in LOG_GROUPS}
        self.columns = {name: [v for v, _ in variables]
                        for name, _, variables in LOG_GROUPS}
        self.log_configs = {}
        self.events = []

    def _make_callback(self, name):
        columns = self.columns[name]

        def on_data(timestamp, data, logconf):
            # timestamp is the drone's clock in ms; host_time_s is this PC's clock
            # since the recorder was created. Both are saved so sheets can be aligned.
            self.rows[name].append(
                [timestamp, round(time.time() - self.t0, 4)] + [data[c] for c in columns])

        return on_data

    def start(self, scf):
        for name, period, variables in LOG_GROUPS:
            lg = LogConfig(name=name, period_in_ms=period)
            for var, vtype in variables:
                lg.add_variable(var, vtype)
            scf.cf.log.add_config(lg)

            if not lg.valid:
                print(f"WARNING: log block '{name}' is not valid "
                      f"(a variable is missing from the firmware?). Skipping it.")
                continue

            lg.data_received_cb.add_callback(self._make_callback(name))
            lg.error_cb.add_callback(
                lambda logconf, msg, n=name: print(f"Log error in '{n}': {msg}"))
            lg.start()
            self.log_configs[name] = lg

    def stop(self):
        for lg in self.log_configs.values():
            try:
                lg.stop()
                lg.delete()
            except Exception as e:
                print("Could not stop log block cleanly:", repr(e))
        self.log_configs.clear()

    def mark(self, label):
        """Record a labelled event (takeoff, landing, ...) with the host time."""
        t = round(time.time() - self.t0, 4)
        self.events.append([t, label])
        print(f"[{t:8.3f} s] {label}")

    def _merged_rows(self):
        """Build one table: one row per CLOCK_GROUP sample, with the latest known
        value of every other group carried forward onto it."""
        clock = self.rows[CLOCK_GROUP]
        others = [n for n in self.rows if n != CLOCK_GROUP]
        other_times = {n: [r[1] for r in self.rows[n]] for n in others}

        table = []
        for t_ms, t_host, *clock_vals in clock:
            row = [t_ms, t_host, ""] + clock_vals
            for n in others:
                i = bisect_right(other_times[n], t_host) - 1   # latest sample at or before t_host
                if i >= 0:
                    row += self.rows[n][i][2:]
                else:
                    row += [None] * len(self.columns[n])       # no sample received yet
            table.append(row)

        # Attach each event to the first row at or after it (or the last row)
        if table:
            row_times = [r[1] for r in table]
            for t, label in self.events:
                i = min(bisect_left(row_times, t), len(table) - 1)
                table[i][2] = f"{table[i][2]}; {label}" if table[i][2] else label
        return table

    def save(self, path):
        names = [CLOCK_GROUP] + [n for n in self.rows if n != CLOCK_GROUP]
        header = ["timestamp_ms", "host_time_s", "event"]
        for n in names:
            header += self.columns[n]

        wb = Workbook()
        ws = wb.active
        ws.title = "Flight"
        ws.append(header)
        table = self._merged_rows()
        for row in table:
            ws.append(row)
        ws.freeze_panes = "A2"

        wb.save(path)
        counts = ", ".join(f"{n}: {len(r)}" for n, r in self.rows.items())
        print(f"Saved {path}  ({len(table)} rows; raw samples {counts})")


class KillSwitchTriggered(Exception):
    """Raised inside the flight code when the kill switch has been pressed."""


class KillSwitch:
    """Emergency stop on the keyboard: Space or Esc (or Ctrl+C) while a flight is running.

    The keys are read from the terminal, so the terminal window must have focus.
    Pressing one cuts the motors immediately from the key-listener thread, without
    waiting for the flight code, which is usually blocked in a sleep.
    """

    def __init__(self):
        self.triggered = threading.Event()
        self.busy = threading.Event()       # set while a flight is running
        self._lock = threading.Lock()
        self._cf = None
        self._mc = None
        self._recorder = None

    # --- used by the flight code ---------------------------------------------------
    def check(self):
        """Raise if the kill switch has been pressed."""
        if self.triggered.is_set():
            raise KillSwitchTriggered()

    def arm(self, cf, mc, recorder):
        self.check()                        # pressed while connecting -> never take off
        self._cf, self._mc, self._recorder = cf, mc, recorder

    def disarm(self):
        # Safety net: if the button was pressed while take-off was still starting, the
        # hover thread may have been created after the first stop. Stop it again.
        if self.triggered.is_set():
            self._cut_motors()
        self._cf = self._mc = self._recorder = None

    def reset(self):
        self.triggered.clear()

    def sleep(self, seconds):
        """Like time.sleep, but wakes up at once and raises if the kill switch is pressed."""
        # Wait in short slices: on Windows a long Event.wait() blocks Ctrl+C handling.
        end = time.monotonic() + seconds
        while True:
            left = end - time.monotonic()
            if left <= 0:
                return
            if self.triggered.wait(min(left, 0.05)):
                raise KillSwitchTriggered()

    # --- the actual kill -----------------------------------------------------------
    def trigger(self, reason="kill switch"):
        with self._lock:
            first = not self.triggered.is_set()
            self.triggered.set()
        if first and self._cf is not None:
            print(f"!!! KILL SWITCH ({reason}) !!!")
            if self._recorder is not None:
                self._recorder.mark(f"KILL SWITCH ({reason})")
            self._cut_motors()

    def _cut_motors(self):
        cf, mc = self._cf, self._mc
        if cf is None:
            return

        def stop_hover_thread():
            # The thread keeps re-sending hover setpoints, which would restart the
            # motors right after a stop command, so it has to go first.
            if mc is not None:
                mc._is_flying = False       # mc.land() becomes a no-op, further commands fail fast
                th = mc._thread
                if th is not None and th.is_alive():
                    th.stop()

        def send_stops():
            for _ in range(3):
                cf.commander.send_stop_setpoint()

        def emergency_stop():
            if KILL_USES_EMERGENCY_STOP:
                cf.loc.send_emergency_stop()

        for step in (stop_hover_thread, send_stops, emergency_stop):
            try:
                step()
            except Exception as e:
                print(f"Kill switch step {step.__name__} failed:", repr(e))

    # --- the keys ------------------------------------------------------------------
    def install(self):
        """Start listening for the kill keys and take over Ctrl+C. Call from the main thread."""

        def on_sigint(signum, frame):
            if self.busy.is_set():
                self.trigger("Ctrl+C")      # during a flight: kill instead of exiting
            else:
                raise KeyboardInterrupt     # at the prompt: quit as usual

        signal.signal(signal.SIGINT, on_sigint)

        try:
            import msvcrt
        except ImportError:
            print("Note: the Space/Esc kill keys need Windows. Ctrl+C still works as a kill switch.")
            return

        def listen():
            # Only read the keyboard while a flight is running, so the
            # "Press Enter to fly" prompt keeps its own keystrokes.
            while True:
                if self.busy.is_set() and msvcrt.kbhit():
                    ch = msvcrt.getwch()
                    if ch == " ":
                        self.trigger("space bar")
                    elif ch == "\x1b":
                        self.trigger("Esc")
                else:
                    time.sleep(0.01)

        threading.Thread(target=listen, daemon=True).start()


def set_param(scf, name, value):
    """Set a firmware parameter. Failures are printed instead of raised so a failed
    write (e.g. after an emergency stop) never breaks the cleanup code."""
    try:
        scf.cf.param.set_value(name, str(value))
    except Exception as e:
        print(f"Could not set {name}:", repr(e))


def make_localmin_callback(recorder):
    """Marks each table edge reported by the firmware in the Excel event column."""
    last = {"count": None}

    def on_data(timestamp, data, logconf):
        c = data["localMin.count"]
        if last["count"] is not None and c != last["count"]:
            what = "entered table" if data["localMin.overTable"] else "left table"
            recorder.mark(f"{what} (target {data['localMin.target']:.3f} m)")
        last["count"] = c

    return on_data


def check_uri():
    available = crtp.scan_interfaces()
    if not available:
        print("No Crazyflies found during scan.")
        return

    for uri, name in available:
        print(f"Available URI: {uri} ({name})")


def main(kill):
    recorder = FlightRecorder()
    filename = f"flight_{datetime.now():%Y%m%d_%H%M%S}.xlsx"

    try:
        print(f"Connecting to {URI}...")
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
            recorder.start(scf)
            try:
                # Create the commander first so the kill switch can reach it
                # even while take-off (which blocks for several seconds) is running.
                mc = MotionCommander(scf, default_height=1.25)
                kill.arm(scf.cf, mc, recorder)

                recorder.mark("takeoff")
                kill.check()
                with mc:
                    recorder.mark("hovering")

                    llg = recorder.log_configs.get("LocalMin")
                    lm_cb = make_localmin_callback(recorder)
                    if llg is not None:
                        llg.data_received_cb.add_callback(lm_cb)
                    else:
                        print("WARNING: no LocalMin log block")

                    try:
                        # Threshold first, so the first enabled tick already uses it.
                        # The firmware detector starts fresh when enabled.
                        set_param(scf, "localMin.stepThresh", STEP_THRESH_M)
                        set_param(scf, "localMin.enable", 1)
                        recorder.mark(f"localMin enabled (stepThresh {STEP_THRESH_M} m)")

                        kill.sleep(5.0)

                        # Same as mc.forward(), but the wait can be interrupted
                        recorder.mark(f"forward {FORWARD_M} m")
                        mc.start_forward()
                        kill.sleep(FORWARD_M / MotionCommander.VELOCITY)
                        mc.stop()

                        kill.sleep(10.0)
                    finally:
                        # Disable before landing, otherwise the override would keep
                        # holding the table height while the descent starts.
                        set_param(scf, "localMin.enable", 0)
                        if llg is not None:
                            llg.data_received_cb.remove_callback(lm_cb)
                        recorder.mark("aborted" if kill.triggered.is_set() else "landing")

                recorder.mark("landed")
            finally:
                kill.disarm()
                recorder.stop()
    finally:
        # Always write whatever was collected, even if the flight failed midway
        recorder.save(filename)


def fly_loop(kill):
    while True:
        try:
            input("Press Enter to fly (Ctrl+C to quit). During a flight, Space or Esc = KILL... ")
        except EOFError:
            return
        check_uri()
        kill.reset()
        kill.busy.set()
        try:
            main(kill)
        except Exception:
            if kill.triggered.is_set():
                print("Flight aborted by the kill switch.")
                if KILL_USES_EMERGENCY_STOP:
                    print("Power-cycle the Crazyflie before flying again.")
            else:
                print("Flight failed:")
                traceback.print_exc()
        finally:
            kill.busy.clear()


if __name__ == "__main__":
    crtp.init_drivers()

    kill = KillSwitch()
    kill.install()

    try:
        fly_loop(kill)
    except KeyboardInterrupt:
        print("Exiting")