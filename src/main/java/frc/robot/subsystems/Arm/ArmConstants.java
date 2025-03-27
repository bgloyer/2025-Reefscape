package frc.robot.subsystems.Arm;

public class ArmConstants {

    public static final int LeftMotorId = 12; 
    public static final int RightMotorId = 13; 
    public static final double MinAngle = -110;
    public static final double MaxAngle = 110;

    // from block cad - degrees
    public static final double AlgaeStore = 0;
    public static final double Store = 4;
    public static final double Station = 43;
    ;//40.966
    public static final double StationOffset = 51.2;//51.2
    public static final double L1 = -52.07;//-72.9
    public static final double L2 = -19; //-19
    public static final double L3 = -24;
    public static final double L4 = -27; //27
    public static final double TopAlgGrab = -30;
    public static final double TopDealgRoll = -29;
    public static final double BottomDealgRoll = -32;
    public static final double BottomAlgGrab = -32;
    public static final double Net = 0;
    public static final double GroundIntake = 21.27; 
    public static final double Climb = -109;

    public static final double kP = 0.09; 
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = -0.2;
    public static final double MaxVelocity = 120; // degrees per sec
    public static final double MaxAcceleration = 500; // degrees per sec^2 500
    public static final double Tolerance = 5;
    public static final double MotorReduction = (7.0 / 68.0) / 4.0 * 360.0;
    public static final double OffsetL2 = -40;//-40
    public static final double OffsetL3 = -40;//-40
    public static final double OffsetL4 = -42;//-42
    public static final double GroundAlgae = -100;
    public static final double OffsetTopAlgGrab = -40.03;    
    public static final double OffsetBotAlgGrab = -40.03;
    public static final double OffsetBotAlgRoll = -61.19;
    public static final double FlooralHandOff = 4;
}
