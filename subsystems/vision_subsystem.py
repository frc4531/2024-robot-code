import ntcore
import wpilib

from networktables import NetworkTables
from commands2 import SubsystemBase


class VisionSubsystem(SubsystemBase):
    # Create a new VisionSubsystem
    current_shoot_x = 0.0
    current_shoot_y = 0.0
    current_shoot_a = 0.0
    current_shoot_v = 0.0

    def __init__(self) -> None:
        super().__init__()
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.shoot_table = self.inst.getTable("limelight-shoot")

        self.shoot_x_entry = self.shoot_table.getEntry("tx")
        self.shoot_y_entry = self.shoot_table.getEntry("ty")
        self.shoot_a_entry = self.shoot_table.getEntry("ta")
        self.shoot_v_entry = self.shoot_table.getEntry("tv")

    def periodic(self) -> None:
        self.current_shoot_x = self.shoot_x_entry.getDouble(0.0)
        self.current_shoot_y = self.shoot_y_entry.getDouble(0.0)
        self.current_shoot_a = self.shoot_a_entry.getDouble(0.0)
        self.current_shoot_v = self.shoot_v_entry.getDouble(0.0)

        wpilib.SmartDashboard.putNumber("Shoot X Entry", self.current_shoot_x)
        wpilib.SmartDashboard.putNumber("Shoot Y Entry", self.current_shoot_y)
        wpilib.SmartDashboard.putNumber("Shoot A Entry", self.current_shoot_a)
        wpilib.SmartDashboard.putNumber("Shoot V Entry", self.current_shoot_v)