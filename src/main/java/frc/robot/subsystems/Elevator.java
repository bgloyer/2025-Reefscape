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
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Configs.ElevatorConfig;

public class Elevator extends SubsystemBase {
    
    private SparkFlex m_leftMotor;
    private SparkFlex m_rightMotor; // follower motor
    private SparkClosedLoopController m_controller;
    private RelativeEncoder m_encoder;
    private double targetPosition;

    public Elevator() {
        m_leftMotor = new SparkFlex(ElevatorConstants.leftMotorId, MotorType.kBrushless);
        m_leftMotor.configure(ElevatorConfig.leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor = new SparkFlex(ElevatorConstants.rightMotorId, MotorType.kBrushless);
        m_rightMotor.configure(ElevatorConfig.rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_controller = m_leftMotor.getClosedLoopController();
        m_encoder = m_leftMotor.getEncoder();
        m_encoder.setPosition(0);
    }
    
    public void setTarget(double height) {
        targetPosition = MathUtil.clamp(height, ElevatorConstants.MinHeight, ElevatorConstants.MaxHeight); 
        m_controller.setReference(targetPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ElevatorConstants.kG);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public boolean onTarget() {
        return Math.abs(m_encoder.getPosition() - targetPosition) < ElevatorConstants.Tolerance;
    }    

    public boolean almostOnTarget() {
        return m_encoder.getPosition() > targetPosition * ElevatorConstants.ApproachingTargetThreshold;
    }
}
