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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Configs;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.Configs.ElevatorConfig;

public class Elevator extends SubsystemBase {
    
    private final SparkFlex m_leftMotor;
    private final SparkFlex m_rightMotor; // follower motor
    private final SparkClosedLoopController m_controller;
    private final RelativeEncoder m_encoder;
    private double targetPosition;

    public Elevator() {
        m_leftMotor = new SparkFlex(ElevatorConstants.leftMotorId, MotorType.kBrushless);
        m_leftMotor.configure(ElevatorConfig.leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor = new SparkFlex(ElevatorConstants.rightMotorId, MotorType.kBrushless);
        m_rightMotor.configure(ElevatorConfig.rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_controller = m_leftMotor.getClosedLoopController();
        m_encoder = m_leftMotor.getEncoder();
        m_encoder.setPosition(0);
        SmartDashboard.putNumber("Elevator kP", ElevatorConstants.kP);
        SmartDashboard.putNumber("Elevator kI", ElevatorConstants.kI);
        SmartDashboard.putNumber("Elevator kD", ElevatorConstants.kD);
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

    public Command updateFromDashboard() {
        return runOnce(() -> {
            Configs.ElevatorConfig.leftMotorConfig.closedLoop.p(SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kP));
            Configs.ElevatorConfig.leftMotorConfig.closedLoop.i(SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kI));
            Configs.ElevatorConfig.leftMotorConfig.closedLoop.d(SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kD));
            Configs.ElevatorConfig.rightMotorConfig.closedLoop.p(SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kP));
            Configs.ElevatorConfig.rightMotorConfig.closedLoop.i(SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kI));
            Configs.ElevatorConfig.rightMotorConfig.closedLoop.d(SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kD));
            m_leftMotor.configure(Configs.ElevatorConfig.leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            m_rightMotor.configure(Configs.ElevatorConfig.leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", m_encoder.getPosition());
    }
}
