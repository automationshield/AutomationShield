"""PlateShield-only serial interface (Python 3.9+).

Upload examples/PlateShield/PlateShield_Python/PlateShield_Python.ino first.
Install: python -m pip install -r requirements.txt
Close Arduino Serial Monitor, then use:

    from plateshield import PlateClass

    with PlateClass("COM11") as PlateShield:
        sample = PlateShield.sensorRead()
        print(sample.x, sample.y)  # mm, currently integer sensor resolution
        reference = PlateShield.referenceRead()  # potentiometer in percent (0-100)
        PlateShield.actuatorWrite(1.0, -1.0)  # servo offsets, NOT position targets
        PlateShield.stop()              # nominal neutral position

Repeat actuatorWrite() within 500 ms to maintain an output. No background keepalive:
if the controller stalls, firmware returns to neutral. Single-threaded API.
Serial implementation follows https://pyserial.readthedocs.io/en/latest/pyserial_api.html
"""

from dataclasses import dataclass
import math
import time

import serial


@dataclass(frozen=True)
class Sample:
    time_ms: int
    x: float
    y: float
    u_x: float
    u_y: float


class PlateClass:
    def __init__(self, port: str, timeout: float = 0.25):
        if not math.isfinite(timeout) or not 0 < timeout <= 1:
            raise ValueError("timeout must be in (0, 1] seconds")
        self._serial = serial.Serial(
            port, 115200, timeout=timeout, write_timeout=timeout
        )
        try:
            # Opening a serial port can reset an Arduino. Allow boot + begin().
            time.sleep(2.5)
            self._serial.reset_input_buffer()
            if self._exchange("HELLO") != "PLATESHIELD 1":
                raise RuntimeError("Upload the PlateShield_Python sketch first")
        except BaseException:
            self._serial.close()
            raise

    def _exchange(self, command: str) -> str:
        try:
            self._serial.write((command + "\n").encode("ascii"))
            response = self._serial.read_until(b"\n", size=160)
            if not response.endswith(b"\n"):
                raise TimeoutError("Incomplete PlateShield reply; reconnect")
            line = response.decode("ascii").strip()
            if line.startswith("ERR "):
                raise RuntimeError(line)
            return line
        except BaseException:
            # Do not reuse an unsynchronised connection after a timeout.
            self._serial.close()
            raise

    def _sample(self, command: str) -> Sample:
        line = self._exchange(command)
        try:
            fields = line.split()
            if len(fields) != 6 or fields[0] != "DATA":
                raise ValueError("Unexpected reply")
            timestamp = int(fields[1])
            values = [float(value) for value in fields[2:]]
            if not 0 <= timestamp <= 0xFFFFFFFF or not all(map(math.isfinite, values)):
                raise ValueError("Invalid measurement")
            return Sample(timestamp, *values)
        except ValueError as exc:
            self._serial.close()
            raise RuntimeError(f"Invalid PlateShield reply: {line!r}") from exc

    def sensorRead(self) -> Sample:
        """Read position and last commanded outputs; does not renew watchdog."""
        return self._sample("READ")

    def actuatorWrite(self, u_x: float, u_y: float) -> Sample:
        """Apply offsets in [-10, 10] degrees and return a fresh measurement."""
        u_x, u_y = float(u_x), float(u_y)
        if not all(math.isfinite(u) and -10 <= u <= 10 for u in (u_x, u_y)):
            raise ValueError("Servo offsets must be finite and within [-10, 10]")
        return self._sample(f"SET {u_x:.4f} {u_y:.4f}")

    def referenceRead(self) -> float:
        """Read potentiometer in percent (0-100), not the circle's target position.

        Requires firmware with REF support; does not renew the actuator watchdog.
        """
        line = self._exchange("REF")
        try:
            fields = line.split()
            if len(fields) != 2 or fields[0] != "REF":
                raise ValueError("Unexpected reference reply")
            value = float(fields[1])
            if not math.isfinite(value) or not 0 <= value <= 100:
                raise ValueError("Invalid potentiometer value")
            return value
        except ValueError as exc:
            self._serial.close()
            raise RuntimeError(f"Invalid PlateShield reply: {line!r}") from exc

    def read(self) -> Sample:
        """Compatibility alias for sensorRead()."""
        return self.sensorRead()

    def write(self, u_x: float, u_y: float) -> Sample:
        """Compatibility alias for actuatorWrite()."""
        return self.actuatorWrite(u_x, u_y)

    def stop(self) -> Sample:
        """Command neutral servo positions, without detaching the servos."""
        return self._sample("STOP")

    def close(self):
        if self._serial.is_open:
            try:
                self.stop()
            finally:
                self._serial.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if exc_type is None:
            self.close()
        else:
            try:
                self.close()
            except Exception:
                pass  # Preserve the original controller/connection error.


class PlateController:
    """Subclass controller(t, dt, reference, sample) to implement a PC controller.

    run() opens the board, schedules samples and returns in-memory history.
    Timing is best-effort on the PC, not hard real-time. No files are generated.
    reference is a callable: reference(t) -> (x_mm, y_mm).
    """

    def __init__(self, port: str):
        self.port = port
        self.history = []

    def reset(self, sample: Sample):
        """Initialise estimator/controller state from the first measurement."""

    def controller(self, t, dt, reference, sample):
        raise NotImplementedError

    def run(self, reference, duration=20.0, frequency=20.0):
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("duration must be positive and finite")
        if not math.isfinite(frequency) or not 5 <= frequency <= 50:
            raise ValueError("frequency must be between 5 and 50 Hz")
        period = 1.0 / frequency
        self.history = []
        with PlateClass(self.port) as PlateShield:
            previous = PlateShield.sensorRead()
            self.reset(previous)
            start = time.monotonic()
            deadline = start + period
            elapsed = 0.0
            while True:
                time.sleep(max(0.0, deadline - time.monotonic()))
                if time.monotonic() - start >= duration:
                    break
                sample = PlateShield.sensorRead()
                # Unsigned difference handles the Arduino millis() rollover.
                dt = ((sample.time_ms - previous.time_ms) & 0xFFFFFFFF) / 1000.0
                if not 0 < dt <= 0.3:
                    raise RuntimeError("Sampling interrupted or board reset; control stopped")
                elapsed += dt
                target = tuple(reference(elapsed))
                if len(target) != 2 or not all(math.isfinite(v) for v in target):
                    raise ValueError("reference must return two finite coordinates in mm")
                u_x, u_y = self.controller(elapsed, dt, target, sample)
                # Do not send a stale command after a slow controller callback.
                if time.monotonic() - start >= duration:
                    break
                if time.monotonic() - deadline > 0.3:
                    raise RuntimeError("Controller missed its deadline; control stopped")
                applied = PlateShield.actuatorWrite(u_x, u_y)
                self.history.append((elapsed, dt, sample.x, sample.y,
                                     target[0], target[1], applied.u_x, applied.u_y))
                previous = sample
                deadline += period
                if deadline <= time.monotonic():
                    # Skip missed slots rather than issuing a burst of commands.
                    deadline = time.monotonic() + period
        return self.history
