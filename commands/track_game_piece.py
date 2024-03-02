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

        self.close_camera_y = -20
        self.far_camera_y = 20
        self.close_speed = 0.1
        self.far_speed = 0.8

        self.cam_range = abs(self.close_camera_y - self.far_camera_y)
        self.speed_range = abs(self.far_speed - self.close_speed)
        self.cam_speed_ratio = self.speed_range / self.cam_range

        self.driver_controller = stick
        self.max_rot_speed = 0.8
        self.min_rot_speed = 0.05

        self.rot_controller = wpimath.controller.PIDController(0.024, 0.01, 0)

        self.speed_controller = wpimath.controller.PIDController(0, 0, 0)

    def initalize(self) -> None:
        self.rot_controller.setTolerance(2)

    def execute(self) -> None:
        target_speed = 0.36

        if self.vision_sub.current_intake_v == 1:
            if self.close_camera_y > self.vision_sub.current_intake_y > self.far_camera_y:
                target_speed = (((self.vision_sub.current_intake_y-self.far_camera_y) * self.angle_range) / self.cam_range) + self.far_angle
                wpilib.SmartDashboard.putNumber("Target Speed", target_speed)
                if self.close_speed < target_speed < self.far_speed:
                    target_speed = self.speed_controller.calculate(self.pivot_sub.get_position(), target_speed)
                    self.drive_sub.xSpeed(target_speed)
                else:
                    self.drive_sub.xSpeed(0)

        # ---- END ANGLE BLOCK, START TURN BLOCK
        pid_output = self.rot_controller.calculate(self.vision_sub.current_intake_x, 0)
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
        return self.intake_sub.intake_prox.get()

    def end(self, interrupted: bool) -> None:
        self.drive_sub.xSpeed(0)
