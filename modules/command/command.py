"""
Decision-making logic.
"""

import math

from pymavlink import mavutil

from ..common.modules.logger import logger
from ..telemetry import telemetry


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on recieved telemetry,
    and send out commands based upon the data.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> tuple[bool, Command]:
        """
        Falliable create (instantiation) method to create a Command object.
        """
        return True, cls(cls.__private_key, connection, target, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        self.connection = connection
        self.target = target
        self.__logger = local_logger
        self.report_count = 0
        self.total_velocity_x = 0.0
        self.total_velocity_y = 0.0
        self.total_velocity_z = 0.0
        self.__logger.info("Created Command")

    def run(self, td: telemetry.TelemetryData) -> tuple[bool, list[str]]:
        """
        Make a decision based on received telemetry data.
        """
        # Log average velocity for this trip so far
        self.total_velocity_x += td.x_velocity
        self.total_velocity_y += td.y_velocity
        self.total_velocity_z += td.z_velocity

        self.report_count += 1

        average_velocity = (
            self.total_velocity_x / self.report_count,
            self.total_velocity_y / self.report_count,
            self.total_velocity_z / self.report_count,
        )

        self.__logger.info(f"Average velocity: {average_velocity}")

        # Use COMMAND_LONG (76) message, assume the target_system=1 and target_componenet=0
        # The appropriate commands to use are instructed below
        outputs = []

        # Adjust height using the comand MAV_CMD_CONDITION_CHANGE_ALT (113)
        # String to return to main: "CHANGE_ALTITUDE: {amount you changed it by, delta height in meters}"
        delta_height = self.target.z - td.z
        if abs(delta_height) > 0.5:
            # COMMAND_LONG:
            # target_system, target_component, command, confirmation,
            # param1, param2, param3, param4, param5, param6, param7
            self.connection.mav.command_long_send(
                1,
                0,
                mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                0,  # confirmation
                1.0,  # param1: rate (m/s)
                0,  # param2: unused
                0,  # param3: unused
                0,  # param4: unused
                0,  # param5: unused
                0,  # param6: unused
                self.target.z,  # param7: target altitude
            )
            outputs.append(f"CHANGE_ALTITUDE: {delta_height}")

        # Adjust direction (yaw) using MAV_CMD_CONDITION_YAW (115). Must use relative angle to current state
        # String to return to main: "CHANGING_YAW: {degree you changed it by in range [-180, 180]}"
        # Positive angle is counter-clockwise as in a right handed system
        dx = self.target.x - td.x
        dy = self.target.y - td.y

        target_yaw = math.degrees(math.atan2(dy, dx))
        delta_yaw = (target_yaw - math.degrees(td.yaw) + 180) % 360 - 180

        if abs(delta_yaw) > 5:
            direction = -1 if delta_yaw > 0 else 1
            # MAVLink yaw commands normally use a positive magnitude in param1,
            # with param3 specifying direction.

            self.connection.mav.command_long_send(
                1,
                0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                abs(delta_yaw),  # angle magnitude
                45,  # yaw speed: arbitrary for this assignment
                direction,  # verify project convention: + means clockwise in MAVLink
                1,  # relative angle adjustment
                0,
                0,
                0,
            )
            outputs.append(f"CHANGING_YAW: {delta_yaw}")

        return True, outputs


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
