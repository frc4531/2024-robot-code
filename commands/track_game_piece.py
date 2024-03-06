import math

import commands2
import wpilib
import wpimath.controller

from constants.swerve_constants import OIConstants
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from subsystems.drivesubsystem import DriveSubsystem


class TrackGamePiece(commands2.CommandBase):

    def __init__(self, vision_sub: VisionSubsystem, drive_sub: DriveSubsystem,
                 intake_sub: IntakeSubsystem, stick: wpilib.Joystick) -> None:
        super().__init__()

        self.vision_sub = vision_sub
        self.drive_sub = drive_sub
        self.intake_sub = intake_sub
        self.addRequirements(self.vision_sub, self.drive_sub, self.intake_sub)

        self.driver_controller = stick
        self.max_rot_speed = 0.8
        self.min_rot_speed = 0.05

        self.alignment_tolerance = 7.5
        self.forward_speed = 0.2

        self.rot_controller = wpimath.controller.PIDController(0.011, 0, 0)

    def initialize(self) -> None:
        self.rot_controller.setTolerance(0.2)

    def execute(self) -> None:
        self.intake_sub.set_intake_speed(0.7)

        if self.vision_sub.current_intake_v == 1 and self.vision_sub.current_intake_y < 20:
            pid_output = self.rot_controller.calculate(self.vision_sub.current_intake_x, 0)
            z_output = max(min(pid_output, self.max_rot_speed), -self.max_rot_speed)

            if 0 < z_output < self.min_rot_speed:
                z_output = self.min_rot_speed
            elif -self.min_rot_speed < z_output < 0:
                z_output = -self.min_rot_speed

            if self.vision_sub.current_intake_x == 1:
                z_output = max(min(pid_output, self.max_rot_speed), -self.max_rot_speed)

            if -self.alignment_tolerance < self.vision_sub.current_intake_x < self.alignment_tolerance:
                y_output = self.forward_speed
            else:
                y_output = -self.driver_controller.getY()

        else:
            z_output = -self.driver_controller.getZ() * 0.5
            y_output = self.driver_controller.getY()

        self.robotDrive.drive(
            -wpimath.applyDeadband(
                ((y_output * math.sin(self.robotDrive.getHeading() * (math.pi / 180))) +
                 (self.driverController.getX() * math.cos(self.robotDrive.getHeading() * (math.pi / 180)))) * 0.5,
                OIConstants.kDriveDeadband
            ),
            -wpimath.applyDeadband(
                ((-y_output * math.cos(self.robotDrive.getHeading() * (math.pi / 180))) +
                 (self.driverController.getX() * math.sin(self.robotDrive.getHeading() * (math.pi / 180)))) * 0.5,
                OIConstants.kDriveDeadband
            ),
            -wpimath.applyDeadband(
                z_output, OIConstants.kDriveTurnDeadband
            ),
            True,
            False,
        )
    def isFinished(self) -> bool:
        return self.intake_sub.intake_prox.get()

    def end(self, interrupted: bool) -> None:
        self.intake_sub.set_intake_speed(0)
