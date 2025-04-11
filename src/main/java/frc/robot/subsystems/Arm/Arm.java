package frc.robot.subsystems.Arm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Configs;
import frc.robot.subsystems.TorSubsystemBase;

public class Arm extends TorSubsystemBase {
    private final SparkFlex m_leftMotor;
    private final SparkFlex m_rightMotor; // follower moter
    private final SparkClosedLoopController m_controller;
    private final RelativeEncoder m_encoder;
    private final TrapezoidProfile m_TrapezoidProfile = new TrapezoidProfile(new Constraints(ArmConstants.MaxVelocity, ArmConstants.MaxAcceleration));
    private State currentState = new State(0,0);
    private State targetState = new State(0,0);
    
        public Arm() {
            m_leftMotor = new SparkFlex(ArmConstants.LeftMotorId, MotorType.kBrushless);
            m_rightMotor = new SparkFlex(ArmConstants.RightMotorId, MotorType.kBrushless);
            m_leftMotor.configure(Configs.ArmConfig.leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_rightMotor.configure(Configs.ArmConfig.rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_controller = m_leftMotor.getClosedLoopController();
            m_encoder = m_leftMotor.getEncoder();
    
            setZero();
            resetSetpoint();
        }
    
        public void resetSetpoint() {
            currentState.position = getAngle();
            // targetState.position = currentState.position;
        }
    
        public void setTargetAngle(double angle) {
            double targetAngle = MathUtil.clamp(angle, ArmConstants.MinAngle, ArmConstants.MaxAngle); 
            if(targetAngle != targetState.position) {
                targetState = new State(targetAngle, 0);
                currentState = new State(getAngle(), currentState.velocity);
            }
        }
    
        private double calculateFeedForward() {
            return ArmConstants.kG * Math.sin(Math.toRadians(m_encoder.getPosition()));
        }
    
        public boolean onTarget() {
            return Math.abs(m_encoder.getPosition() - targetState.position) < ArmConstants.Tolerance;
        }
    
        public void setZero() {
            if (m_leftMotor.getAbsoluteEncoder().getPosition() <= 45)
                m_encoder.setPosition(m_leftMotor.getAbsoluteEncoder().getPosition());
            else 
                m_encoder.setPosition(m_leftMotor.getAbsoluteEncoder().getPosition() - 90);   
        }
    
        public double getAngle() {
            return m_encoder.getPosition();
        }
    
        @Override
        public void periodic() {
            if (!DriverStation.isTestEnabled()) {
                currentState = m_TrapezoidProfile.calculate(0.02, currentState, targetState);
                m_controller.setReference(currentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, calculateFeedForward());
            }
            SmartDashboard.putNumber("Arm Angle", m_encoder.getPosition());
            SmartDashboard.putBoolean("Arm on target", onTarget());
        }
    
        public void stopPid() {
            m_controller.setReference(0, ControlType.kVoltage);
        }

        @Override
        public void toggleIdleMode() {
            SparkBaseConfig config = super.getIdleModeConfig();
            m_leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            m_rightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        @Override
        public boolean isBrakeMode() {
            return m_leftMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
        }

}
