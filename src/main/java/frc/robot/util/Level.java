package frc.robot.util;

import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Claw.ClawConstants.WristConstants;
import frc.robot.subsystems.Elevator.ElevatorConstants;

public enum Level {
    
    NET(ElevatorConstants.Net, ArmConstants.Net, WristConstants.AlgaeStore, false),
    ALGAESTORE(ElevatorConstants.AlgaeStore, ArmConstants.AlgaeStore, WristConstants.AlgaeStore, false),
    STORE(ElevatorConstants.Store, ArmConstants.Store, WristConstants.Store, false), 
    ONE(ElevatorConstants.L1, ArmConstants.L1, WristConstants.L1, true),
    TWO(ElevatorConstants.L2, ArmConstants.L2, WristConstants.L2, true),
    THREE(ElevatorConstants.L3, ArmConstants.L3, WristConstants.L3, true),
    FOUR(ElevatorConstants.L4, ArmConstants.L4, WristConstants.L4, true),
    OFFSETTWO(ElevatorConstants.OffsetL2, ArmConstants.OffsetL2, WristConstants.OffsetL2, true),
    OFFSETTHREE(ElevatorConstants.OffsetL3, ArmConstants.OffsetL3, WristConstants.OffsetL3, true),
    OFFSETFOUR(ElevatorConstants.OffsetL4, ArmConstants.OffsetL4, WristConstants.OffsetL4, true),
    TOPALGAEGRAB(ElevatorConstants.TopAlgGrab, ArmConstants.TopAlgGrab, WristConstants.TopAlgGrab, false), 
    OFFSETTOPALGAEGRAB(ElevatorConstants.OffsetTopAlgGrab, ArmConstants.OffsetTopAlgGrab, WristConstants.OffsetTopAlgGrab, false), 
    TOPALGAEROLL(ElevatorConstants.TopDealgRoll, ArmConstants.TopDealgRoll, WristConstants.TopDealgRoll, false),
    BOTTOMALGAEGRAB(ElevatorConstants.BottomAlgGrab, ArmConstants.BottomAlgGrab, WristConstants.BottomAlgGrab, false), 
    OFFSETBOTTOMALGAEGRAB(ElevatorConstants.OffsetBotAlgGrab, ArmConstants.OffsetBotAlgGrab, WristConstants.OffsetBotAlgGrab, false), 
    BOTTOMALGAEROLL(ElevatorConstants.BottomDealgRoll, ArmConstants.BottomDealgRoll, WristConstants.BottomAlgaeRoll, true),
    OFFSETBOTTOMALGAEROLL(ElevatorConstants.OffsetBotAlgRoll, ArmConstants.OffsetBotAlgRoll, WristConstants.OffsetBotAlgRoll, true),
    GROUNDALGAE(ElevatorConstants.Store, ArmConstants.GroundAlgae, WristConstants.AlgaeStore, false),
    FLOORALHANDOFF(ElevatorConstants.FlooralHandOff, ArmConstants.FlooralHandOff, WristConstants.FlooralHandOff, false);

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
            case TOPALGAEGRAB:
                return Helpers.isOneCoralAway ? Level.OFFSETTOPALGAEGRAB : Level.TOPALGAEGRAB;
            case BOTTOMALGAEGRAB:
                return Helpers.isOneCoralAway ? Level.OFFSETBOTTOMALGAEGRAB : Level.BOTTOMALGAEGRAB;
            case BOTTOMALGAEROLL:
                return Helpers.isOneCoralAway ? Level.OFFSETBOTTOMALGAEROLL : Level.BOTTOMALGAEROLL;
            default:
                return this;
        }
    }
}