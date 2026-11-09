import logging
import time

import cflib.crtp as crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig

URI = "radio://0/80/2M"


def main():
    print(f"Connecting to {URI}...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        print("Taking off...")
        with MotionCommander(scf, default_height=1.25) as mc:
            lg = attach_step_compensation(scf, mc)

            time.sleep(5.0)
            
            print("Moving forward...")
            mc.forward(2.0)
            
            time.sleep(10.0)
            
            lg.stop()
            lg.delete()
            print("Landing...")


def check_uri():
    available = crtp.scan_interfaces()
    if not available:
        print("No Crazyflies found during scan.")
        return

    for uri, name in available:
        print(f"Available URI: {uri} ({name})")

STEP_THRESHOLD_M = 0.15     # ignore changes smaller than this
MAX_VALID_RANGE_M = 3.0     # ignore out-of-range readings from the sensor

def attach_step_compensation(scf, mc):
    last = {'z': None}

    def on_data(timestamp, data, logconf):
        z = data['range.zrange'] / 1000.0          # mm to m
        print(f"t={timestamp} ms  zrange={z} mm ({z / 1000.0:.3f} m)")
        if z <= 0.0 or z > MAX_VALID_RANGE_M:
            return                                  # invalid reading
        if last['z'] is not None:
            step = z - last['z']
            if abs(step) > STEP_THRESHOLD_M:
                print(f"Height change: {last['z']:.3f} m -> {z:.3f} m "
                  f"(step {step:+.3f} m) at t={timestamp} ms")
                try:
                    mc.adjust_height(z)
                    print(f"setpoint now {mc._thread.get_height():.3f} m")
                except Exception:
                    print("adjust_height failed:", repr(e))
        last['z'] = z

    lg = LogConfig(name='zrange', period_in_ms=10)
    lg.add_variable('range.zrange', 'uint16_t')
    scf.cf.log.add_config(lg)
    lg.data_received_cb.add_callback(on_data)
    lg.start()
    return lg


if __name__ == "__main__":
    #logging.basicConfig(level=logging.INFO)
    crtp.init_drivers()

    try:
        while True:
            input("Press Enter to fly (Ctrl+C to quit)... ")
            check_uri()
            main()
    except KeyboardInterrupt:
        print("Exiting")