package frc.robot.util;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClawConstants.WristConstants;
import frc.robot.constants.ElevatorConstants;

public enum Level {
    
    NET(ElevatorConstants.Net, ArmConstants.Net, WristConstants.Net, false),
    ALGAESTORE(ElevatorConstants.AlgaeStore, ArmConstants.AlgaeStore, WristConstants.AlgaeStore, false),
    STORE(ElevatorConstants.Store, ArmConstants.Store, WristConstants.Store, false), 
    ONE(ElevatorConstants.L1, ArmConstants.L1, WristConstants.L1, true),
    TWO(ElevatorConstants.L2, ArmConstants.L2, WristConstants.L2, true),
    THREE(ElevatorConstants.L3, ArmConstants.L3, WristConstants.L3, true),
    FOUR(ElevatorConstants.L4, ArmConstants.L4, WristConstants.L4, true),
    OFFSETTWO(ElevatorConstants.OffsetL2, ArmConstants.OffsetL2, WristConstants.OffsetL2, true),
    OFFSETTHREE(ElevatorConstants.OffsetL3, ArmConstants.OffsetL3, WristConstants.OffsetL3, true),
    OFFSETFOUR(ElevatorConstants.OffsetL4, ArmConstants.OffsetL4, WristConstants.OffsetL4, true),
    TOPALGAE(0.613, -55, 140, true), // 0.61
    // TOPALGAE(ElevatorConstants.TopDealg, ArmConstants.TopDealg, WristConstants.TopDealg, true), // 0.61
    BOTTOMALGAE(ElevatorConstants.BottomDealg, ArmConstants.BottomDeAlg, WristConstants.L3, true);

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

    public Level withOffset() {
        switch (this) {
            case TWO:
                return Helpers.isOneCoralAway ?  Level.OFFSETTWO : Level.TWO;
            case THREE:
                return Helpers.isOneCoralAway ?  Level.OFFSETTHREE : Level.THREE;
            case FOUR:
                return Helpers.isOneCoralAway ?  Level.OFFSETFOUR : Level.FOUR;
            default:
                return this;
        }
    }
}