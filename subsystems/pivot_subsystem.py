import rev
import wpilib
from commands2 import SubsystemBase

from constants.pivot_constants import PivotConstants


class PivotSubsystem(SubsystemBase):
    # Create a new PivotSubsystem

    def __init__(self) -> None:
        super().__init__()

        self.pivot_motor = rev.CANSparkFlex(3, rev.CANSparkFlex.MotorType.kBrushless)

        self.pivot_encoder = wpilib.DutyCycleEncoder(channel=0)
        self.pivot_encoder.setPositionOffset(PivotConstants.kPivotOffset)
        self.pivot_encoder.setDutyCycleRange(0, 1)

        self.upper_limit_switch = wpilib.DigitalInput(2)
        self.lower_limit_switch = wpilib.DigitalInput(3)

    def periodic(self) -> None:
        wpilib.SmartDashboard.putNumber("Pivot Encoder Value", self.get_position())
        wpilib.SmartDashboard.putBoolean("Upper Encoder Value", self.upper_limit_switch.get())
        wpilib.SmartDashboard.putBoolean("Lower Encoder Value", self.lower_limit_switch.get())


    def get_position(self):
        return self.pivot_encoder.getAbsolutePosition()

    def set_pivot_speed(self, speed):
        if speed > 0 and self.upper_limit_switch.get():
            self.pivot_motor.set(speed)
        elif speed < 0 and self.lower_limit_switch.get():
            self.pivot_motor.set(speed)
        else:
            self.pivot_motor.set(0)
