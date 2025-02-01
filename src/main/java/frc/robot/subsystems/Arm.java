package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final SparkFlex m_leftMotor;
    private final SparkFlex m_rightMotor; // follower moter
    private final SparkClosedLoopController m_controller;
    private final AbsoluteEncoder m_encoder;
    private double targetAngle;

    public Arm() {
        m_leftMotor = new SparkFlex(ArmConstants.LeftMotorId, MotorType.kBrushless);
        m_rightMotor = new SparkFlex(ArmConstants.RightMotorId, MotorType.kBrushless);
        m_controller = m_leftMotor.getClosedLoopController();
        m_encoder = m_leftMotor.getAbsoluteEncoder();
    }

    public void setTargetAngle(double angle) {
        
        targetAngle = MathUtil.clamp(angle, ArmConstants.MinAngle, ArmConstants.MaxAngle); 
        m_controller.setReference(targetAngle, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0 , calculateFeedForward()); // may need to run in periodic
    }

    private double calculateFeedForward() {
        return ArmConstants.kG * Math.sin(Math.toRadians(m_encoder.getPosition()));
    }

    public boolean onTarget() {
        return Math.abs(m_encoder.getPosition() - targetAngle) < ArmConstants.Tolerance;
    }
}
