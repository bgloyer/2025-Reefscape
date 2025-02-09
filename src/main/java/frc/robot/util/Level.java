package frc.robot.util;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClawConstants.WristConstants;
import frc.robot.constants.ElevatorConstants;

public enum Level {
    
    STORE(ElevatorConstants.Store, ArmConstants.Store, WristConstants.Store), 
    ONE(ElevatorConstants.L1, ArmConstants.L1, WristConstants.L1),
    TWO(ElevatorConstants.L2, ArmConstants.L2, WristConstants.L2),
    THREE(ElevatorConstants.L3, ArmConstants.L3, WristConstants.L3),
    FOUR(ElevatorConstants.L4, ArmConstants.L4, WristConstants.L4);

    public double elevatorHeight;
    public double armAngle;
    public double wristAngle;

    Level(double elevatorHeight, double armAngle, double wristAngle) {
        this.elevatorHeight = elevatorHeight; 
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
    }
}