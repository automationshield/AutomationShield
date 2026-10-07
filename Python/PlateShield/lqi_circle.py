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

from plateshield import PlateController

DEFAULT_PORT = "COM11"  # Used when running from the editor without arguments.


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class LQIController(PlateController):
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
    if args.plot:
        import matplotlib.pyplot as plt

    def reference(t):
        angle = 2.0 * math.pi * t / args.circle_period
        return 51.0 + 15.0 * math.cos(angle), 30.0 + 15.0 * math.sin(angle)

    controller = LQIController(args.port)
    print(f"LQI on PC, port {args.port}, 20 Hz. Press Ctrl+C to stop.")
    try:
        controller.run(reference, duration=args.duration, frequency=20.0)
    except KeyboardInterrupt:
        print("Experiment interrupted.")
    history = controller.history
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
