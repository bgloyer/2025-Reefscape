package frc.robot.constants;

public class ArmConstants {

    public static final int LeftMotorId = 12; 
    public static final int RightMotorId = 13; 
    public static final double MinAngle = -100;
    public static final double MaxAngle = 90;

    // from block cad - degrees
    public static final double Store = -10;
    public static final double Station = 35;
    public static final double L1 = -105.702;
    public static final double L2 = -27.28;
    public static final double L3 = -29;
    public static final double L4 = -35;
    public static final double DeAlgaeL3 = -28;
    public static final double Net = 0;
    public static final double GroundIntake = 21.27; 

    public static final double kP = 0.09;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = -0.2;
    public static final double MotorReduction = (7.0 / 68.0) / 4.0 * 360.0;
    public static final double Tolerance = 5;
    public static final double SlewRate = 1.7; // degrees per 0.02 

}
