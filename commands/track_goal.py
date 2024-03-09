import math

import commands2
import wpilib
import wpimath.controller

from constants.swerve_constants import OIConstants
from subsystems.led_subsystem import LedSubsystem
from subsystems.pivot_subsystem import PivotSubsystem
from subsystems.shooter_subsystem import ShooterSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from subsystems.drivesubsystem import DriveSubsystem


class TrackGoal(commands2.CommandBase):

    def __init__(self, vision_sub: VisionSubsystem, drive_sub: DriveSubsystem, pivot_sub: PivotSubsystem,
                 led_sub: LedSubsystem, stick: wpilib.Joystick) -> None:
        super().__init__()

        self.vision_sub = vision_sub
        self.drive_sub = drive_sub
        self.pivot_sub = pivot_sub
        self.addRequirements(self.vision_sub, self.drive_sub, self.pivot_sub)

        self.close_camera_y = 20.5
        self.far_camera_y = -9.2
        self.close_angle = 0.4
        self.far_angle = 0.342

        self.cam_range = abs(self.close_camera_y - self.far_camera_y)
        self.angle_range = abs(self.close_angle-self.far_angle)
        self.cam_angle_ratio = self.angle_range / self.cam_range

        self.driver_controller = stick
        self.max_rot_speed = 0.8
        self.min_rot_speed = 0.05

        self.rot_controller = wpimath.controller.PIDController(0.024, 0.01, 0)

        self.angle_controller = wpimath.controller.PIDController(7.5, 0, 0)

    def initialize(self) -> None:
        self.rot_controller.setTolerance(0.1)
        # self.shooter_sub.set_velocities(-6000, 6500)

    def execute(self) -> None:
        target_angle = 0.36

        if self.vision_sub.current_shoot_v == 1:
            if self.close_camera_y > self.vision_sub.current_shoot_y > self.far_camera_y:
                target_angle = (((self.vision_sub.current_shoot_y-self.far_camera_y) * self.angle_range) / self.cam_range) + self.far_angle
                wpilib.SmartDashboard.putNumber("Target Angle", target_angle)
                if self.far_angle < target_angle < self.close_angle:
                    angle_output = self.angle_controller.calculate(self.pivot_sub.get_position(), target_angle)
                    self.pivot_sub.set_pivot_speed(angle_output)
                else:
                    self.pivot_sub.set_pivot_speed(0)

        # ---- END ANGLE BLOCK, START TURN BLOCK
        if self.vision_sub.current_shoot_v == 1:
            pid_output = self.rot_controller.calculate(self.vision_sub.current_shoot_x, 0)
            z_output = max(min(pid_output, self.max_rot_speed), -self.max_rot_speed)

            if 0 < z_output < self.min_rot_speed:
                z_output = self.min_rot_speed
            elif -self.min_rot_speed < z_output < 0:
                z_output = -self.min_rot_speed

            if self.vision_sub.current_shoot_x == 1:
                z_output = max(min(pid_output, self.max_rot_speed), -self.max_rot_speed)
        else:
            z_output = self.driver_controller.getZ() * 0.5

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
                -z_output, OIConstants.kDriveTurnDeadband
            ),
            True,
            False),
    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.pivot_sub.set_pivot_speed(0)
        # self.shooter_sub.stop_shooter()
