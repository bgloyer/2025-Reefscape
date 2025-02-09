package frc.robot.constants;

public class ClawConstants {

    public final class WristConstants {
        public static final int MotarCanId = 14;
        public static final double MotorGearReduction = 12.0 / 64.0;
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;
        public static final double Tolerance = 5;
        public static final double SlewRate = 4;

        // from block cad - degrees
        public static final double Initial = 0;
        public static final double Store = 90;
        public static final double Station = 30;
        public static final double L1 = 90;
        public static final double L2 = 83.19;
        public static final double L3 = 85;
        public static final double L4 = 47;
        public static final double DeAlgaeL3 = 55;
        public static final double Net = 150;
        public static final double GroundIntake = 21.27;

        public static final double MaxAngle = 150;
        public static final double MinAngle = 0;
    }

    public final class CoralIntakeConstants {
        public static final int MotorCanId = 15;
        public static final double kP = 0.0007;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double IntakeVoltage = -3;
        public static final double OuttakeVoltage = 3;
        public static final double IntakeVelocityFF = 1.0 / Constants.kNeoVortexkV / 12.0;
    }
}
