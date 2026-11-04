from __future__ import annotations

import time

import serial


class RobotConnection:
    """Newline-delimited serial connection to the Arduino controller."""

    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 1.0) -> None:
        if not port.strip():
            raise ValueError("A serial port name is required.")
        if baudrate <= 0 or timeout <= 0:
            raise ValueError("Baud rate and timeout must be greater than zero.")
        self._connection = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(2)

    @property
    def is_open(self) -> bool:
        return self._connection.is_open

    def send_command(self, command: str) -> None:
        if command not in {"STOP", "STATUS", "SENSORS"}:
            raise ValueError(f"Unsupported robot command: {command!r}")
        self._connection.write(f"{command}\n".encode("ascii"))

    def read_message(self) -> str | None:
        response = self._connection.readline()
        if not response:
            return None
        return response.decode("ascii", errors="strict").strip()

    def close(self) -> None:
        self._connection.close()

    def __enter__(self) -> RobotConnection:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
