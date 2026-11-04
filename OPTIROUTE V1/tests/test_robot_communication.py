import unittest
from unittest.mock import patch

from robot.communication import RobotConnection


class RobotConnectionTests(unittest.TestCase):
    @patch("robot.communication.time.sleep")
    @patch("robot.communication.serial.Serial")
    def test_sensor_diagnostics_command_is_sent(self, serial_factory, _sleep) -> None:
        connection = RobotConnection("/dev/test")

        connection.send_command("SENSORS")

        serial_factory.return_value.write.assert_called_once_with(b"SENSORS\n")
        self.assertTrue(connection.is_open)

    @patch("robot.communication.time.sleep")
    @patch("robot.communication.serial.Serial")
    def test_motion_commands_are_rejected_by_pi_client(
        self, _serial_factory, _sleep
    ) -> None:
        connection = RobotConnection("/dev/test")

        for command in ("START", "FORWARD", "BACKWARD", "LEFT", "RIGHT"):
            with self.subTest(command=command):
                with self.assertRaises(ValueError):
                    connection.send_command(command)


if __name__ == "__main__":
    unittest.main()
