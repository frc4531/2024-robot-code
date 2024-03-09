import rev
import wpilib
from commands2 import SubsystemBase


class LedSubsystem(SubsystemBase):

    def __init__(self):
        super.__init__()

        self.led_motor = wpilib.Spark(0)
        self.current_led_value = 0.67

    def set_color(self, speed):
        if self.current_led_value != speed:
            self.led_motor.set(speed)
            self.current_led_value = speed


