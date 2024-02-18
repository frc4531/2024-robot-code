import math

import commands2
import wpilib
import wpimath.controller

from constants.swerve_constants import OIConstants
from subsystems.vision_subsystem import VisionSubsystem
from subsystems.drivesubsystem import DriveSubsystem


class TrackGoal(commands2.CommandBase):

    def __init__(self, vision_sub: VisionSubsystem, drive_sub: DriveSubsystem, stick: wpilib.Joystick) -> None:
        super().__init__()

        self.vision_sub = vision_sub
        self.drive_sub = drive_sub
        self.addRequirements(self.vision_sub,self.drive_sub)

        self.driver_controller = stick
        self.max_rot_speed = 0.8
        self.min_rot_speed = 0.05

        self.rot_controller = wpimath.controller.PIDController(0.024,0.01,0)

    def initalize(self) -> None:
        self.rot_controller.setTolerance(2)

    def execute(self) -> None:
        pid_output = self.rot_controller.calculate(self.vision_sub.current_shoot_x, 0)
        z_output = max(min(pid_output, self.max_rot_speed), -self.max_rot_speed)

        if 0 < z_output < self.min_rot_speed:
            z_output = self.min_rot_speed
        elif -self.min_rot_speed < z_output < 0:
            z_output = -self.min_rot_speed

        if self.vision_sub.current_shoot_x == 1:
            z_output = max(min(pid_output, self.max_rot_speed), -self.max_rot_speed)

        self.drive_sub.drive(
            -wpimath.applyDeadband(
                (self.driver_controller.getY() * math.sin(self.drive_sub.getHeading() * (math.pi / 180))) +
                (self.driver_controller.getX() * math.cos(self.drive_sub.getHeading() * (math.pi / 180))),
                OIConstants.kDriveDeadband
            ),
            -wpimath.applyDeadband(
                (-self.driver_controller.getY() * math.cos(self.drive_sub.getHeading() * (math.pi / 180))) +
                (self.driver_controller.getX() * math.sin(self.drive_sub.getHeading() * (math.pi / 180))),
                OIConstants.kDriveDeadband
            ),
            -wpimath.applyDeadband(
                -z_output, OIConstants.kDriveDeadband
            ),
            False,
            False),
    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        test = 0
