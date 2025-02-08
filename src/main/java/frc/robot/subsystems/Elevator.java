package frc.robot.subsystems;

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
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.Configs.ElevatorConfig;

public class Elevator extends SubsystemBase {
    
    private final SparkFlex m_leftMotor;
    private final SparkFlex m_rightMotor; // follower motor
    private final SparkClosedLoopController m_controller;
    private final RelativeEncoder m_encoder;
    private double targetPosition;
    private double currentSetpoint;

    public Elevator() {
        m_leftMotor = new SparkFlex(ElevatorConstants.leftMotorId, MotorType.kBrushless);
        m_leftMotor.configure(ElevatorConfig.leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor = new SparkFlex(ElevatorConstants.rightMotorId, MotorType.kBrushless);
        m_rightMotor.configure(ElevatorConfig.rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_controller = m_leftMotor.getClosedLoopController();
        m_encoder = m_leftMotor.getEncoder();
        m_encoder.setPosition(0);
        resetSetpoint();
    }

    public void resetSetpoint() {
        currentSetpoint = getHeight();
        targetPosition = currentSetpoint;
    }

    
    public void setTarget(double height) {
        targetPosition = MathUtil.clamp(height, ElevatorConstants.MinHeight, ElevatorConstants.MaxHeight); 
        currentSetpoint = getHeight();
    }

    
    public double getHeight() {
        return m_encoder.getPosition();
    }
    
    public boolean onTarget() {
        return Math.abs(m_encoder.getPosition() - targetPosition) < ElevatorConstants.Tolerance;
    }    
    
    public boolean almostOnTarget() {
        return m_encoder.getPosition() > targetPosition * ElevatorConstants.ApproachingTargetThreshold;
    }


    private void positionSlewRateLimiting() {
        m_controller.setReference(0, ControlType.kDutyCycle);
        // double error = targetPosition - currentSetpoint;
        // currentSetpoint += Math.min(Math.abs(error), ElevatorConstants.SlewRate) * Math.signum(error);
        // m_controller.setReference(currentSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.kG);
    }

    @Override
    public void periodic() {
        positionSlewRateLimiting();
        SmartDashboard.putNumber("Elevator Encoder", m_encoder.getPosition());
        SmartDashboard.putNumber("Elevator Current", m_leftMotor.getOutputCurrent());
    }
}
