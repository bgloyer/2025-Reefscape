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
    private SparkFlex m_leftMotor;
    private SparkFlex m_rightMotor;
    private SparkClosedLoopController m_controller;
    private AbsoluteEncoder m_Encoder;

    public Arm() {
        m_leftMotor = new SparkFlex(ArmConstants.LeftMotorId, MotorType.kBrushless);
        m_rightMotor = new SparkFlex(ArmConstants.RightMotorId, MotorType.kBrushless);
        m_controller = m_leftMotor.getClosedLoopController();
        m_Encoder = m_leftMotor.getAbsoluteEncoder();
    }

    public void setTarget(double angle) {
        double clampedAngle = MathUtil.clamp(angle, ArmConstants.MinAngle, ArmConstants.MaxAngle); 
        m_controller.setReference(clampedAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0 , calculateFeedForward()); // may need to run in periodic
    }

    private double calculateFeedForward() {
        return ArmConstants.kG * Math.cos(m_Encoder.getPosition() * 2 * Math.PI);
    }
}
