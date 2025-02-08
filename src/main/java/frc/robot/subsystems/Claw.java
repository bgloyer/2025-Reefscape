package frc.robot.subsystems;

import java.io.Console;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.Configs;
import frc.robot.constants.Constants;
import frc.robot.constants.ClawConstants.CoralIntakeConstants;
import frc.robot.constants.ClawConstants.WristConstants;
import frc.robot.constants.Configs.ClawConfig;

public class Claw extends SubsystemBase {
    
    private final SparkFlex m_wristMotor;
    private final SparkFlex m_intakeMotor;
    private final SparkClosedLoopController m_wristController;
    private final SparkClosedLoopController m_intakeController;
    private final RelativeEncoder m_encoder; 
    private double targetAngle;
    private double currentSetpoint;

    public Claw() {
        m_wristMotor = new SparkFlex(WristConstants.MotarCanId, MotorType.kBrushless);
        m_wristMotor.configure(ClawConfig.wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_wristController = m_wristMotor.getClosedLoopController();
        m_encoder = m_wristMotor.getEncoder();
        m_encoder.setPosition(WristConstants.Initial);
        m_wristMotor.configure(ClawConfig.wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_intakeMotor = new SparkFlex(CoralIntakeConstants.MotorCanId, MotorType.kBrushless);
        m_intakeMotor.configure(ClawConfig.intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_intakeController = m_intakeMotor.getClosedLoopController();
        resetSetpoint();
    }

    public void resetSetpoint() {
        currentSetpoint = getAngle();
        targetAngle = currentSetpoint;
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }

    public void runIntake() {
        runVoltage(CoralIntakeConstants.IntakeVoltage);
    }

    public void runOuttake() {
        runVoltage(CoralIntakeConstants.OuttakeVoltage);
    }

    public void runVoltage(double volts) {
        m_intakeController.setReference(volts, ControlType.kVoltage);
    }

    public void stopIntake() {
        m_intakeMotor.stopMotor();
    }

    public void setTargetAngle(double angle) {
        targetAngle = MathUtil.clamp(angle, WristConstants.MinAngle, WristConstants.MaxAngle); 
        currentSetpoint = getAngle();
    }

    private void positionSlewRateLimiting() {
        double error = targetAngle - currentSetpoint;
        currentSetpoint += Math.min(Math.abs(error), WristConstants.SlewRate) * Math.signum(error);
        m_wristController.setReference(currentSetpoint, ControlType.kPosition);
    }

    public boolean onTarget() {
        return Math.abs(m_encoder.getPosition() - targetAngle) < WristConstants.Tolerance;
    }

    @Override
    public void periodic() {
        positionSlewRateLimiting();
        SmartDashboard.putNumber("Wrist Angle", getAngle());
        SmartDashboard.putNumber("Intake Velocity", m_intakeMotor.getEncoder().getVelocity());
    }
}
