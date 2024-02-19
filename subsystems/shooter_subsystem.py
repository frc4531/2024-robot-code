import rev
from commands2 import SubsystemBase


class ShooterSubsystem(SubsystemBase):
    # Create a new VisionSubsystem

    def __init__(self) -> None:
        super().__init__()

        # Shooter motors = 2 spark flex motors
        self.left_shooter_motor = rev.CANSparkFlex(1, rev.CANSparkFlex.MotorType.kBrushless)
        self.right_shooter_motor = rev.CANSparkFlex(2, rev.CANSparkFlex.MotorType.kBrushless)

        # PID controllers for each shooter motor
        self.left_pid_controller = self.left_shooter_motor.getPIDController()
        self.right_pid_controller = self.right_shooter_motor.getPIDController()

        # Encoders for each shooter motor
        self.left_encoder = self.left_shooter_motor.getEncoder()
        self.right_encoder = self.right_shooter_motor.getEncoder()

        # PID Controller values for motor
        self.kP = 0.1
        self.kI = 1e-4
        self.kD = 0
        self.kIz = 0
        self.kFF = 0
        self.kMinOutput = -1
        self.kMaxOutput = 1

        # Motor max RPM
        self.maxRPM = 6700

        # The restoreFactoryDefault() method can be used to the reset the
        # configuration parameters in the SPARK MAX to their factory default
        # state. If no argument is passed, these parameters will not persist
        # between power cycles
        self.left_shooter_motor.restoreFactoryDefaults()
        self.right_shooter_motor.restoreFactoryDefaults()

        # set PID constants
        self.left_pid_controller.setP(self.kP)
        self.left_pid_controller.setI(self.kI)
        self.left_pid_controller.setD(self.kD)
        self.left_pid_controller.setIZone(self.kIz)
        self.left_pid_controller.setFF(self.kP)
        self.left_pid_controller.setOutputRange(self.kMinOutput,self.kMaxOutput)

        self.right_pid_controller.setP(self.kP)
        self.right_pid_controller.setI(self.kI)
        self.right_pid_controller.setD(self.kD)
        self.right_pid_controller.setIZone(self.kIz)
        self.right_pid_controller.setFF(self.kP)
        self.right_pid_controller.setOutputRange(self.kMinOutput, self.kMaxOutput)

        # Sets velocity targets for both shooter motors
    def set_velocities(self, left_velocity,right_velocity):
        self.left_pid_controller.setReference(left_velocity, rev.CANSparkFlex.ControlType.kVelocity)
        self.right_pid_controller.setReference(right_velocity, rev.CANSparkFlex.ControlType.kVelocity)

    def set_percentage_speed(self, left_percent, right_percent):
        self.left_shooter_motor.set(left_percent)
        self.right_shooter_motor.set(right_percent)

    def stop_shooter(self):
        self.left_shooter_motor.stopMotor()
        self.right_shooter_motor.stopMotor()
