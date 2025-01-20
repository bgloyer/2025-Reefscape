package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    
    private SparkFlex m_leftMotor;
    private SparkFlex m_rightMotor;
    private SparkClosedLoopController m_controller;
    private double targetHeight = ElevatorConstants.Level1;

    public Elevator() {
        m_leftMotor = new SparkFlex(ElevatorConstants.leftMotorId, MotorType.kBrushless);
        m_rightMotor = new SparkFlex(ElevatorConstants.rightMotorId, MotorType.kBrushless);
    }
    
    @Override
    public void periodic() {
        m_controller.setReference(targetHeight, ControlType.kMAXMotionPositionControl);
    }
}
