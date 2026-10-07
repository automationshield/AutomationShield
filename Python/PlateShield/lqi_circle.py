"""LQI control on the PC with the gains from PlateShield/LQ_circle_example.

1. Upload PlateShield_Python.ino using Arduino IDE; close Serial Monitor.
2. Install: python -m pip install -r Python/PlateShield/requirements.txt
3. Run directly from the editor (DEFAULT_PORT), or:
   python Python/PlateShield/lqi_circle.py --port COM11
   Optional: --duration 30 --circle-period 8 --plot

Ctrl+C stops the experiment and commands neutral. No files are written.
--plot displays results after stopping; install matplotlib to use it.
Velocity uses a backward difference; the controller runs at nominally 20 Hz.
The gains are copied, not retuned or validated for PC/USB timing.
"""

import argparse
import math
import time

from plateshield import PlateClass

DEFAULT_PORT = "COM11"  # Used when running from the editor without arguments.


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class LQIController:
    def reset(self, sample):
        self.previous_x = sample.x
        self.previous_y = sample.y
        self.integral_x = 0.0
        self.integral_y = 0.0

    def controller(self, t, dt, reference, sample):
        r_x, r_y = reference
        v_x = (sample.x - self.previous_x) / dt
        v_y = (sample.y - self.previous_y) / dt
        self.integral_x = clamp(self.integral_x + dt * (r_x - sample.x), -100, 100)
        self.integral_y = clamp(self.integral_y + dt * (r_y - sample.y), -100, 100)
        u_x = -0.43 * (sample.x - r_x) - 0.11 * v_x + 0.20 * self.integral_x
        u_y = -0.35 * (sample.y - r_y) - 0.10 * v_y + 0.13 * self.integral_y
        self.previous_x, self.previous_y = sample.x, sample.y
        return clamp(u_x, -10, 10), clamp(u_y, -10, 10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--duration", type=float, default=20.0, help="seconds")
    parser.add_argument("--circle-period", type=float, default=8.0, help="seconds per revolution")
    parser.add_argument("--plot", action="store_true", help="plot after stopping (requires matplotlib)")
    args = parser.parse_args()
    if not math.isfinite(args.circle_period) or args.circle_period <= 0:
        parser.error("--circle-period must be positive and finite")
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be positive and finite")
    if args.plot:
        import matplotlib.pyplot as plt

    def reference(t):
        angle = 2.0 * math.pi * t / args.circle_period
        return 51.0 + 15.0 * math.cos(angle), 30.0 + 15.0 * math.sin(angle)

    controller = LQIController()
    history = []
    period = 0.05  # Nominal sampling period: 20 Hz.
    print(f"LQI on PC, port {args.port}, 20 Hz. Press Ctrl+C to stop.")
    try:
        with PlateClass(args.port) as PlateShield:
            previous = PlateShield.sensorRead()
            controller.reset(previous)
            start = time.monotonic()
            deadline = start + period
            elapsed = 0.0

            while True:
                time.sleep(max(0.0, deadline - time.monotonic()))
                if time.monotonic() - start >= args.duration:
                    break

                # Read the ball position from the Arduino.
                sample = PlateShield.sensorRead()
                dt = ((sample.time_ms - previous.time_ms) & 0xFFFFFFFF) / 1000.0
                if not 0 < dt <= 0.3:
                    raise RuntimeError("Sampling interrupted or board reset; control stopped")
                elapsed += dt
                target = reference(elapsed)

                # Compute the LQI commands on the PC.
                u_x, u_y = controller.controller(elapsed, dt, target, sample)
                if time.monotonic() - start >= args.duration:
                    break
                if time.monotonic() - deadline > 0.3:
                    raise RuntimeError("Controller missed its deadline; control stopped")

                # Send both servo commands to the Arduino.
                applied = PlateShield.actuatorWrite(u_x, u_y)
                history.append((elapsed, dt, sample.x, sample.y,
                                target[0], target[1], applied.u_x, applied.u_y))
                previous = sample
                deadline += period
                if deadline <= time.monotonic():
                    deadline = time.monotonic() + period
        # The context manager commands neutral and closes the serial port.
    except KeyboardInterrupt:
        print("Experiment interrupted.")
    print(f"Recorded {len(history)} samples in memory; connection closed.")
    if args.plot and history:
        t, dt, x, y, rx, ry, ux, uy = zip(*history)
        fig, axes = plt.subplots(1, 2)
        axes[0].plot(rx, ry, "--", label="Reference")
        axes[0].plot(x, y, label="Measured")
        axes[0].set(xlabel="x [mm]", ylabel="y [mm]", title="PlateShield LQI")
        axes[0].set_aspect("equal", adjustable="box")
        axes[0].legend()
        axes[1].plot(t, ux, label="uX")
        axes[1].plot(t, uy, label="uY")
        axes[1].set(xlabel="Time [s]", ylabel="Servo offset [deg]")
        axes[1].legend()
        fig.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
