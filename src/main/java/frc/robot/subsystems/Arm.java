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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Configs;

public class Arm extends SubsystemBase {
    private final SparkFlex m_leftMotor;
    private final SparkFlex m_rightMotor; // follower moter
    private final SparkClosedLoopController m_controller;
    private final RelativeEncoder m_encoder;
    private double currentSetpoint;
    private double targetAngle;

    public Arm() {
        m_leftMotor = new SparkFlex(ArmConstants.LeftMotorId, MotorType.kBrushless);
        m_rightMotor = new SparkFlex(ArmConstants.RightMotorId, MotorType.kBrushless);
        m_leftMotor.configure(Configs.ArmConfig.leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor.configure(Configs.ArmConfig.rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_controller = m_leftMotor.getClosedLoopController();
        m_encoder = m_leftMotor.getEncoder();
        resetVortexEncoder();
        resetSetpoint();
    }

    public void resetSetpoint() {
        currentSetpoint = getAngle();
        targetAngle = currentSetpoint;
    }

    public void setTargetAngle(double angle) {
        targetAngle = MathUtil.clamp(angle, ArmConstants.MinAngle, ArmConstants.MaxAngle); 
        currentSetpoint = getAngle();
    }

    private double calculateFeedForward() {
        return ArmConstants.kG * Math.sin(Math.toRadians(m_encoder.getPosition()));
    }

    public boolean onTarget() {
        return Math.abs(m_encoder.getPosition() - targetAngle) < ArmConstants.Tolerance;
    }

    public void resetVortexEncoder() {
        if (m_leftMotor.getAbsoluteEncoder().getPosition() <= 45)
            m_encoder.setPosition(m_leftMotor.getAbsoluteEncoder().getPosition());
        else 
            m_encoder.setPosition(m_leftMotor.getAbsoluteEncoder().getPosition() - 90);   
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }
    
    private void positionSlewRateLimiting() {
        double error = targetAngle - currentSetpoint;
        // velocity error 
        currentSetpoint += Math.min(Math.abs(error), ArmConstants.SlewRate) * Math.signum(error);
        m_controller.setReference(currentSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateFeedForward());
    }

    @Override
    public void periodic() {
        positionSlewRateLimiting();
        SmartDashboard.putNumber("Arm Angle", m_encoder.getPosition());
    }
}
