package frc.robot.util;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClawConstants.WristConstants;
import frc.robot.constants.ElevatorConstants;

public enum Level {
    
    STORE(ElevatorConstants.Store, ArmConstants.Store, WristConstants.Store, false), 
    ONE(ElevatorConstants.L1, ArmConstants.L1, WristConstants.L1, true),
    TWO(ElevatorConstants.L2, ArmConstants.L2, WristConstants.L2, true),
    THREE(ElevatorConstants.L3, ArmConstants.L3, WristConstants.L3, true),
    FOUR(ElevatorConstants.L4, ArmConstants.L4, WristConstants.L4, true),
    TOPALGAE(ElevatorConstants.DeAlgaeL3, ArmConstants.DeAlgaeL3, WristConstants.DeAlgaeL3, true);

    public double elevatorHeight;
    public double armAngle;
    public double wristAngle;
    public boolean isReefScoringPosition;

    Level(double elevatorHeight, double armAngle, double wristAngle, boolean isReefScoringPosition) {
        this.elevatorHeight = elevatorHeight; 
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
        this.isReefScoringPosition = isReefScoringPosition;
    }
}