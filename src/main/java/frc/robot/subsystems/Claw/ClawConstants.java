package frc.robot.subsystems.Claw;

import frc.robot.subsystems.Drive.DriveAutomation.AligningConstants;

public class ClawConstants {

    public final class WristConstants {
        public static final int MotarCanId = 14;
        public static final double MotorGearReduction = 12.0 / 64.0 * 7 / 34;
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;
        public static final double Tolerance = 5;
        public static final double SlewRate = 5.5;
        // from block cad - degrees
        public static final double AlgaeStore = 128; //124.25
        public static final double AlgaeNetFlick = 90;
        public static final double Initial = 0;
        public static final double Store = 90;

        public static final double Station = 9.955;
        // public static final double Station = 30;
        public static final double StationOffset = 2.06;
        public static final double L1 = 3;//161.2
        public static final double L2 = 83.19;//83.19
        public static final double L3 = 85;
        public static final double L4 = 50;//50; //75 
        public static final double TopAlgGrab = 124.25;
        public static final double TopDealgRoll = 85;
        public static final double BottomAlgGrab = 124.25;
        public static final double BottomAlgaeRoll = 88;
        public static final double GroundIntake = 21.27;

        public static final double MaxAngle = 180;
        public static final double MinAngle = 0;
        public static final double OffsetL2 = 96.3;
        public static final double OffsetL3 = 96.3;
        public static final double OffsetL4 = 75;
        public static final double GroundAlgae = AlgaeStore;
        public static final double OffsetTopAlgGrab = 124.25;    
        public static final double OffsetBotAlgGrab = 124.25;
        public static final double OffsetBotAlgRoll = 120.27;
        public static final double FlooralHandOff = 161;
    }

    public final class CoralIntakeConstants {
        public static final int MotorCanId = 15;
        public static final double kP = 0.0007;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double IntakeVoltage = -2.75;
        public static final double ScoreVoltage = -6;
        public static final double OuttakeVoltage = 4;
        public static final double IntakeVelocityFF = 1.0 / AligningConstants.kNeoVortexkV / 12.0;
        public static final double HandOffVoltage = -3; // -3
    }
}