import math

import commands2
import wpilib
import wpimath.controller
from wpilib import SmartDashboard

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
        self.forward_speed = 0.3

        self.rot_controller = wpimath.controller.PIDController(0.011, 0, 0)

        self.prox_has_been_false = False

    def initialize(self) -> None:
        self.rot_controller.setTolerance(0.2)

        SmartDashboard.putBoolean("LED_TrackingGamePiece", True)

    def execute(self) -> None:
        self.intake_sub.set_intake_speed(0.7)

        if self.intake_sub.intake_prox.get():
            self.prox_has_been_false = True

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
            z_output = self.driver_controller.getZ() * 0.5
            y_output = -self.driver_controller.getY()

        if -self.alignment_tolerance < self.vision_sub.current_intake_x < self.alignment_tolerance:
            self.drive_sub.drive(
                -wpimath.applyDeadband(
                    self.driver_controller.getX(), OIConstants.kDriveDeadband
                ),
                -wpimath.applyDeadband(
                    y_output, OIConstants.kDriveDeadband
                ),
                wpimath.applyDeadband(
                    z_output, OIConstants.kDriveDeadband
                ),
                True,
                False,
            )
        else:
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
                wpimath.applyDeadband(
                    z_output, OIConstants.kDriveDeadband
                ),
                True,
                False,
            )

    def isFinished(self) -> bool:
        return self.prox_has_been_false and not self.intake_sub.intake_prox.get()

    def end(self, interrupted: bool) -> None:
        self.intake_sub.set_intake_speed(0)
        SmartDashboard.putBoolean("LED_TrackingGamePiece", False)
        SmartDashboard.putBoolean("LED_NewGamePiece", True)
