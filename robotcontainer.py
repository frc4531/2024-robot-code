import math

import commands2
import commands2.button

import wpimath
import wpilib
from commands2 import cmd, WaitCommand
from wpilib import SmartDashboard
from wpimath._controls._controls.controller import HolonomicDriveController

from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from commands.amp_down import AmpDown
from commands.climber_l_down import ClimberLDown
from commands.climber_l_up import ClimberLUp
from commands.climber_r_down import ClimberRDown
from commands.climber_r_up import ClimberRUp
from commands.climbers_down import ClimbersDown
from commands.climbers_up import ClimbersUp
from commands.drive_along_trajectory import DriveAlongTrajectory
from commands.drive_turn_to_angle import DriveTurnToAngle
# from commands.drive_along_trajectory import DriveAlongTrajectory
from commands.intake_in import IntakeIn
from commands.intake_in_until_loaded import IntakeInUntilLoaded
from commands.intake_out import IntakeOut
from commands.pivot_down import PivotDown
from commands.pivot_up import PivotUp
from commands.pivot_to_position import PivotToPosition
from commands.shooter_amp import ShooterAmp
from commands.amp_up import AmpUp
from commands.shooter_spin_up import ShooterSpinUp
from commands.track_game_piece import TrackGamePiece
from commands.track_goal import TrackGoal
from constants.swerve_constants import AutoConstants, DriveConstants, OIConstants
from subsystems.led_subsystem import LedSubsystem
from subsystems.right_climber_subsystem import RightClimberSubsystem
from subsystems.left_climber_subsystem import LeftClimberSubsystem
from subsystems.intake_subsystem import IntakeSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.pivot_subsystem import PivotSubsystem
from subsystems.shooter_subsystem import ShooterSubsystem
from subsystems.vision_subsystem import VisionSubsystem
from subsystems.amp_subsystem import AmpSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()
        self.intakeSubsystem = IntakeSubsystem()
        self.shooterSubsystem = ShooterSubsystem()
        self.pivotSubsystem = PivotSubsystem()
        self.visionSubsystem = VisionSubsystem()
        self.rightClimberSubsystem = RightClimberSubsystem()
        self.leftClimberSubsystem = LeftClimberSubsystem()
        self.ledSubsystem = LedSubsystem()
        self.ampSubsystem = AmpSubsystem()

        # The driver controllers
        self.driverController = wpilib.Joystick(OIConstants.kDriverControllerPort)
        self.operatorController = wpilib.Joystick(OIConstants.kOperatorControllerPort)

        # Autonomous Chooser Setup
        self.nothingAuto = "Do Nothing"
        self.shootOneOnlyAuto = "Shoot 1 Only"
        self.shootOneLeaveMidAuto = "MID - Shoot 1 + Leave"
        self.shootOneLeaveSidesAuto = "SIDES - Shoot 1 + Leave"
        self.shootTwoMidAuto = "MID - Shoot 2"
        self.shootTwoLeftAuto = "LEFT - Shoot 2"
        self.shootTwoRightAuto = "RIGHT - Shoot 2"
        self.chooser = wpilib.SendableChooser()

        self.chooser.setDefaultOption("Shoot 1 Only", self.shootOneOnlyAuto)
        self.chooser.addOption("Do Nothing", self.nothingAuto)
        self.chooser.addOption("MID - Shoot 1 + Leave", self.shootOneLeaveMidAuto)
        self.chooser.addOption("SIDES - Shoot 1 + Leave", self.shootOneLeaveSidesAuto)
        self.chooser.addOption("MID - Shoot 2", self.shootTwoMidAuto)
        self.chooser.addOption("LEFT - Shoot 2", self.shootTwoLeftAuto)
        self.chooser.addOption("RIGHT - Shoot 2", self.shootTwoRightAuto)
        SmartDashboard.putData("Auto Chooser", self.chooser)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        (self.driverController.getY() * math.sin(self.robotDrive.getHeading() * (math.pi / 180))) +
                        (self.driverController.getX() * math.cos(self.robotDrive.getHeading() * (math.pi / 180))),
                        OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        (-self.driverController.getY() * math.cos(self.robotDrive.getHeading() * (math.pi / 180))) +
                        (self.driverController.getX() * math.sin(self.robotDrive.getHeading() * (math.pi / 180))),
                        OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getZ(), OIConstants.kDriveDeadband
                    ),
                    True,
                    False,
                ),
                self.robotDrive,
            )
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Set wheels to X (brake)
        commands2.button.JoystickButton(self.driverController, 8).toggleOnTrue(
            commands2.RunCommand(
                lambda: self.robotDrive.setX(),
                self.robotDrive,
            )
        )

        # Slow mode
        commands2.button.JoystickButton(self.driverController, 2).toggleOnTrue(
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        ((self.driverController.getY() * math.sin(self.robotDrive.getHeading() * (math.pi / 180))) +
                         (self.driverController.getX() * math.cos(
                             self.robotDrive.getHeading() * (math.pi / 180)))) * 0.5,
                        OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        ((-self.driverController.getY() * math.cos(self.robotDrive.getHeading() * (math.pi / 180))) +
                         (self.driverController.getX() * math.sin(
                             self.robotDrive.getHeading() * (math.pi / 180)))) * 0.5,
                        OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getZ() * 0.5, OIConstants.kDriveTurnDeadband
                    ),
                    True,
                    False,
                ),
                self.robotDrive,
            )
        )
        # Press for Note Vision Tracking
        commands2.button.JoystickButton(self.driverController, 1).onTrue(
            TrackGamePiece(self.visionSubsystem, self.robotDrive, self.intakeSubsystem, self.ledSubsystem,
                           self.driverController)
        )
        # Left Climber Up
        commands2.button.JoystickButton(self.operatorController, 1).whileTrue(
            ClimberLUp(self.leftClimberSubsystem)
        )
        # Left Climber Down
        commands2.button.JoystickButton(self.operatorController, 2).whileTrue(
            ClimberLDown(self.leftClimberSubsystem)
        )
        # Both Climbers Up
        commands2.button.JoystickButton(self.operatorController, 3).whileTrue(
            ClimbersUp(self.rightClimberSubsystem, self.leftClimberSubsystem)
        )
        # Both Climbers Down
        commands2.button.JoystickButton(self.operatorController, 4).whileTrue(
            ClimbersDown(self.rightClimberSubsystem, self.leftClimberSubsystem)
        )
        # Right Climber Up
        commands2.button.JoystickButton(self.operatorController, 5).whileTrue(
            ClimberRUp(self.rightClimberSubsystem)
        )
        # Right Climber Up
        commands2.button.JoystickButton(self.operatorController, 6).whileTrue(
            ClimberRDown(self.rightClimberSubsystem)
        )
        # Hold for Pivot Up
        commands2.button.JoystickButton(self.operatorController, 7).whileTrue(
            PivotUp(self.pivotSubsystem)
        )
        # Hold for Pivot Down
        commands2.button.JoystickButton(self.operatorController, 8).whileTrue(
            PivotDown(self.pivotSubsystem)
        )
        # Toggle Speaker Vision Tracking
        commands2.button.JoystickButton(self.operatorController, 9).toggleOnTrue(
            TrackGoal(self.visionSubsystem, self.robotDrive, self.pivotSubsystem,
                      self.ledSubsystem, self.driverController)
        )
        # Hold for Manual Intake In
        commands2.button.JoystickButton(self.operatorController, 10).whileTrue(
            IntakeIn(self.intakeSubsystem)
        )
        # Toggle Shooter
        commands2.button.JoystickButton(self.operatorController, 11).toggleOnTrue(
            ShooterSpinUp(self.shooterSubsystem)
        )

        # Hold for Deploy Amp Flipper and Slow Shooter and Pivot to Amp Angle
        commands2.button.JoystickButton(self.operatorController, 12).whileTrue(
            AmpUp(self.ampSubsystem)
        )
        commands2.button.JoystickButton(self.operatorController, 12).whileTrue(
            ShooterAmp(self.shooterSubsystem)
        )
        commands2.button.JoystickButton(self.operatorController, 12).onTrue(
            PivotToPosition(self.pivotSubsystem, 0.38)
        )
        # Hold for Retract Amp Flipper
        commands2.button.JoystickButton(self.operatorController, 13).whileTrue(
            AmpDown(self.ampSubsystem)
        )

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # Middle Auto Trajectory to follow. All units in meters.
        mid_trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(0, -2, Rotation2d.fromDegrees(0)),
            config,
        )

        # Left Auto Trajectory to follow. All units in meters.
        sides_trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [],
            # End 3 meters straight ahead of where we started, facing forward
            Pose2d(0, -2, Rotation2d.fromDegrees(0)),
            config,
        )

        thetaController = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        holonomic_controller = HolonomicDriveController(PIDController(AutoConstants.kPXController, 0, 0),
                                                        PIDController(AutoConstants.kPYController, 0, 0),
                                                        thetaController)

        mid_trajectory_command = commands2.SwerveControllerCommand(
            mid_trajectory,
            self.robotDrive.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controllers
            holonomic_controller,
            self.robotDrive.setModuleStates,
            (self.robotDrive,),
        )

        sides_trajectory_command = commands2.SwerveControllerCommand(
            sides_trajectory,
            self.robotDrive.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            # Position controllers
            holonomic_controller,
            self.robotDrive.setModuleStates,
            (self.robotDrive,),
        )

        auto_selected = self.chooser.getSelected()

        match auto_selected:
            case self.nothingAuto:
                return WaitCommand(1)
            case self.shootOneOnlyAuto:
                return commands2.SequentialCommandGroup(
                    commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(1),
                        ShooterSpinUp(self.shooterSubsystem),
                        PivotToPosition(self.pivotSubsystem, 0.41)
                    ),
                    commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(2),
                        ShooterSpinUp(self.shooterSubsystem),
                        PivotToPosition(self.pivotSubsystem, 0.41),
                        IntakeIn(self.intakeSubsystem)
                    )
                )
            case self.shootOneLeaveMidAuto:
                # Reset odometry to the starting pose of the trajectory.
                self.robotDrive.resetOdometry(mid_trajectory.initialPose())

                return commands2.SequentialCommandGroup(
                    commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(1),
                        ShooterSpinUp(self.shooterSubsystem),
                        PivotToPosition(self.pivotSubsystem, 0.41)
                    ),
                    commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(2),
                        ShooterSpinUp(self.shooterSubsystem),
                        PivotToPosition(self.pivotSubsystem, 0.41),
                        IntakeIn(self.intakeSubsystem)
                    ),
                    mid_trajectory_command.andThen(
                        commands2.RunCommand(
                            lambda: self.robotDrive.drive(0, 0, 0, False, False)
                        )
                    )
                )
            case self.shootOneLeaveSidesAuto:
                return commands2.SequentialCommandGroup(
                    commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(1),
                        ShooterSpinUp(self.shooterSubsystem),
                        PivotToPosition(self.pivotSubsystem, 0.41)
                    ),
                    commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(2),
                        ShooterSpinUp(self.shooterSubsystem),
                        PivotToPosition(self.pivotSubsystem, 0.41),
                        IntakeIn(self.intakeSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(0.5),
                        commands2.RunCommand(lambda: self.robotDrive.drive(0, -0.2, 0, True, False))
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(2),
                        DriveTurnToAngle(self.robotDrive, 0)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(2),
                        commands2.RunCommand(lambda: self.robotDrive.resetOdometry(sides_trajectory.initialPose()))
                    ),
                    sides_trajectory_command.andThen(
                        commands2.RunCommand(
                            lambda: self.robotDrive.drive(0, 0, 0, False, False)
                        )
                    )
                )
            case self.shootTwoMidAuto:
                # Reset odometry to the starting pose of the trajectory.
                self.robotDrive.resetOdometry(mid_trajectory.initialPose())

                return commands2.SequentialCommandGroup(
                    commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(1),
                        ShooterSpinUp(self.shooterSubsystem),
                        PivotToPosition(self.pivotSubsystem, 0.41)
                    ),
                    commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(2),
                        ShooterSpinUp(self.shooterSubsystem),
                        PivotToPosition(self.pivotSubsystem, 0.41),
                        IntakeIn(self.intakeSubsystem)
                    ),
                    TrackGamePiece(self.visionSubsystem, self.robotDrive, self.intakeSubsystem,
                                   self.ledSubsystem, self.driverController
                                   ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(2),
                        TrackGoal(self.visionSubsystem, self.robotDrive, self.pivotSubsystem,
                                  self.ledSubsystem, self.driverController
                                  ),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(2),
                        TrackGoal(self.visionSubsystem, self.robotDrive, self.pivotSubsystem,
                                  self.ledSubsystem, self.driverController
                                  ),
                        ShooterSpinUp(self.shooterSubsystem),
                        IntakeIn(self.intakeSubsystem)
                    )
                )
            case self.shootTwoRightAuto:
                return commands2.SequentialCommandGroup(
                    commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(1.5),
                        ShooterSpinUp(self.shooterSubsystem),
                        PivotToPosition(self.pivotSubsystem, 0.41)
                    ),
                    commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(2),
                        ShooterSpinUp(self.shooterSubsystem),
                        PivotToPosition(self.pivotSubsystem, 0.41),
                        IntakeIn(self.intakeSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(0.5),
                        commands2.RunCommand(lambda: self.robotDrive.drive(0, -0.2, 0, True, False)),
                        ShooterSpinUp(self.shooterSubsystem),
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(1.25),
                        DriveTurnToAngle(self.robotDrive, 0),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(0.5),
                        commands2.RunCommand(lambda: self.robotDrive.drive(0, -0.2, 0, True, False)),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        TrackGamePiece(self.visionSubsystem, self.robotDrive, self.intakeSubsystem,
                                       self.ledSubsystem, self.driverController
                                       ),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(1.25),
                        DriveTurnToAngle(self.robotDrive, -30),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(0.5),
                        commands2.RunCommand(lambda: self.robotDrive.drive(0, 0.2, 0, True, False)),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(2),
                        TrackGoal(self.visionSubsystem, self.robotDrive, self.pivotSubsystem,
                                  self.ledSubsystem, self.driverController
                                  ),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(5),
                        TrackGoal(self.visionSubsystem, self.robotDrive, self.pivotSubsystem,
                                  self.ledSubsystem, self.driverController
                                  ),
                        ShooterSpinUp(self.shooterSubsystem),
                        IntakeIn(self.intakeSubsystem)
                    )
                )
            case self.shootTwoLeftAuto:
                return commands2.SequentialCommandGroup(
                    commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(1.5),
                        ShooterSpinUp(self.shooterSubsystem),
                        PivotToPosition(self.pivotSubsystem, 0.41)
                    ),
                    commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(2),
                        ShooterSpinUp(self.shooterSubsystem),
                        PivotToPosition(self.pivotSubsystem, 0.41),
                        IntakeIn(self.intakeSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(0.5),
                        commands2.RunCommand(lambda: self.robotDrive.drive(0, -0.2, 0, True, False)),
                        ShooterSpinUp(self.shooterSubsystem),
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(1.25),
                        DriveTurnToAngle(self.robotDrive, 0),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(0.5),
                        commands2.RunCommand(lambda: self.robotDrive.drive(0, -0.2, 0, True, False)),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        TrackGamePiece(self.visionSubsystem, self.robotDrive, self.intakeSubsystem,
                                       self.ledSubsystem, self.driverController
                                       ),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(1.25),
                        DriveTurnToAngle(self.robotDrive, 30),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(0.5),
                        commands2.RunCommand(lambda: self.robotDrive.drive(0, 0.2, 0, True, False)),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(2),
                        TrackGoal(self.visionSubsystem, self.robotDrive, self.pivotSubsystem,
                                  self.ledSubsystem, self.driverController
                                  ),
                        ShooterSpinUp(self.shooterSubsystem)
                    ),
                    commands2.ParallelDeadlineGroup(
                        WaitCommand(5),
                        TrackGoal(self.visionSubsystem, self.robotDrive, self.pivotSubsystem,
                                  self.ledSubsystem, self.driverController
                                  ),
                        ShooterSpinUp(self.shooterSubsystem),
                        IntakeIn(self.intakeSubsystem)
                    )
                )
