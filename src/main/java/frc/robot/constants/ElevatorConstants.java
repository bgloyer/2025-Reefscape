package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class ElevatorConstants {
    public static final int leftMotorId = 10;
    public static final int rightMotorId = 11;
    
    public static final double kP = 3.8;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0.37;
    public static final double MaxVelocity = 3; // meters per second
    public static final double MaxAcceleration = 6; // meters per second^2 3.5
    
    public static final double MinHeight = 0;
    public static final double MaxHeight = Units.inchesToMeters(50);
    public static final double Tolerance = 0.02;
    public static final double ApproachingTargetThreshold = 0.5;

    // from block cad
    
    public static final double AlgaeStore = 0.37;
    public static final double Store = 0.0;
    // public static final double Station = Units.inchesToMeters(1) * 2;
    public static final double Station = 0;
    public static final double StationOffset = 0.085;
    public static final double L1 = 0; // 0.3
    public static final double L2 = Units.inchesToMeters(0.4) * 2;
    public static final double L3 = Units.inchesToMeters(9) * 2;
    public static final double L4 = Units.inchesToMeters(24) * 2; // 1.25
    public static final double NotTippable = 1.2;
    public static final double TopAlgGrab = 0.3876;
    public static final double TopDealgRoll = 0.79756;
    public static final double BottomAlgGrab = 0.05;
    public static final double BottomDealgRoll = Units.inchesToMeters(7.75) * 2;
    public static final double Net = Units.inchesToMeters(25) * 2;
    

    public static final double DiameterMeters = Units.inchesToMeters(1.751); // ask alex what this should be
    public static final double MotorReduction = 58.0 / 7.0;
    public static final double OffsetL2 = 0.098;
    public static final double OffsetL3 = 0.503;
    public static final double OffsetL4 = 1.25;
}