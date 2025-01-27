package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.ClawConstants.CoralIntakeConstants;
import frc.robot.constants.ClawConstants.WristConstants;
import frc.robot.constants.Configs.ClawConfig;

public class Claw extends SubsystemBase {
    
    private final SparkFlex m_wristMotor;
    private final SparkFlex m_intakeMotor;
    private final SparkClosedLoopController m_wristController;
    private final SparkClosedLoopController m_intakeController;
    private final RelativeEncoder m_encoder; 

    public Claw() {
        m_wristMotor = new SparkFlex(WristConstants.MotarCanId, MotorType.kBrushless);
        m_wristMotor.configure(ClawConfig.wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_wristController = m_wristMotor.getClosedLoopController();
        m_encoder = m_wristMotor.getEncoder();
        m_encoder.setPosition(WristConstants.Initial);

        m_intakeMotor = new SparkFlex(CoralIntakeConstants.MotorCanId, MotorType.kBrushless);
        m_wristMotor.configure(ClawConfig.intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_intakeController = m_intakeMotor.getClosedLoopController();
    }

    public void runIntake() {
        setTargetVelocity(CoralIntakeConstants.IntakeVelocity);
    }

    public void runOuttake() {
        setTargetVelocity(CoralIntakeConstants.OuttakeVelocity);
    }

    public void stopIntake() {
        m_intakeMotor.stopMotor();
    }

    public void setTargetAngle(double angle) {
        double clampedAngle = MathUtil.clamp(angle, WristConstants.MinAngle, WristConstants.MaxAngle); 
        m_wristController.setReference(clampedAngle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, calculateFeedForward());
    }

    private double calculateFeedForward() {
        return WristConstants.kG * Math.sin(Math.toRadians(m_encoder.getPosition()));
    }

    private void setTargetVelocity(double velocity) {
        m_intakeController.setReference(velocity, ControlType.kVelocity);
    }
}
