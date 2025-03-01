package frc.robot.constants;

public class ClawConstants {

    public final class WristConstants {
        public static final int MotarCanId = 14;
        public static final double MotorGearReduction = 12.0 / 64.0 * 16 / 23;
        public static final double kP = 0.3;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;
        public static final double Tolerance = 5;
        public static final double SlewRate = 5.5;

        // from block cad - degrees
        public static final double AlgaeStore = 129.3;
        public static final double Initial = 0;
        public static final double Store = 90;
        public static final double Station = 30;
        public static final double StationOffset = 0;
        public static final double L1 = 161.2;
        public static final double L2 = 83.19;
        public static final double L3 = 85;
        public static final double L4 = 47; //75 
        public static final double TopDealg = 85;
        public static final double Net = 150;
        public static final double GroundIntake = 21.27;

        public static final double MaxAngle = 180;
        public static final double MinAngle = 0;
        public static final double OffsetL2 = 96.3;
        public static final double OffsetL3 = 96.3;
        public static final double OffsetL4 = 75;
    }

    public final class CoralIntakeConstants {
        public static final int MotorCanId = 15;
        public static final double kP = 0.0007;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double IntakeVoltage = -3
        ;
        public static final double ScoreVoltage = -6;
        public static final double OuttakeVoltage = 4;
        public static final double IntakeVelocityFF = 1.0 / Constants.kNeoVortexkV / 12.0;
    }
}