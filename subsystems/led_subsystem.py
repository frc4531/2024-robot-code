import rev
import wpilib
from commands2 import SubsystemBase
from wpilib import AddressableLED


class LedSubsystem(SubsystemBase):

    def __init__(self) -> None:
        super().__init__()

        self.led_motor = wpilib.Spark(0)
        self.current_led_value = 0

    #def periodic(self) -> None:
        #self.set_color(0.65) # Rainbow Barf: -0.45

    def set_color(self, speed):
        if self.current_led_value != speed:
            self.led_motor.set(speed)
            self.current_led_value = speed
