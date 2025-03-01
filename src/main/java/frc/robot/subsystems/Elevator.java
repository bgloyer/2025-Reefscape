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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.util.Level;
import frc.robot.constants.Configs.ElevatorConfig;

public class Elevator extends SubsystemBase {
    
    private final SparkFlex m_leftMotor;
    private final SparkFlex m_rightMotor; // follower motor
    private final SparkClosedLoopController m_controller;
    private final RelativeEncoder m_encoder;
    private final TrapezoidProfile m_TrapezoidProfile = new TrapezoidProfile(new Constraints(ElevatorConstants.MaxVelocity, ElevatorConstants.MaxAcceleration));
    private State targetState = new State(0, 0);
    private State currentState = new State(0, 0);

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
        currentState = new State(getHeight(), 0);
        targetState = currentState;
    }

    
    public void setTarget(double height) {
        double targetPosition = MathUtil.clamp(height, ElevatorConstants.MinHeight, ElevatorConstants.MaxHeight); 
        if(targetPosition != targetState.position) {
            targetState = new State(targetPosition, 0);
            currentState = new State(getHeight(),m_encoder.getVelocity());
        }
    }

    
    public double getHeight() {
        return m_encoder.getPosition();
    }
    
    public boolean onTarget() {
        return Math.abs(m_encoder.getPosition() - targetState.position) < ElevatorConstants.Tolerance;
    }    
    
    public boolean almostAtStore() {
        return Math.abs(m_encoder.getPosition() - targetState.position) < ElevatorConstants.NotTippable;
    }

    public boolean approachingHeight(Level level) {
        switch (level) {
            case FOUR:
                return Math.abs(m_encoder.getPosition() - targetState.position) < 0.55;
            case THREE:
                return Math.abs(m_encoder.getPosition() - targetState.position) < 0.4;
            default:
                return onTarget();
        }
    }

    @Override
    public void periodic() {
        currentState = m_TrapezoidProfile.calculate(0.02, currentState, targetState);
        m_controller.setReference(currentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.kG);
        SmartDashboard.putNumber("Left Elevator Encoder", m_encoder.getPosition());
        SmartDashboard.putNumber("Right Elevator Encoder", m_rightMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator vel", m_encoder.getVelocity());
        SmartDashboard.putNumber("Elevator Current", m_leftMotor.getOutputCurrent());
    }
}
