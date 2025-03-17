package frc.robot.Climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Configs.ClimberConfig;

public class Climber extends SubsystemBase {
    
    private final SparkFlex m_motor; 
    private final AbsoluteEncoder m_encoder; 
    private final SparkClosedLoopController m_controller; 
    private double targetAngle = ClimbConstants.StoreAngle;
    
    public Climber() {
        m_motor = new SparkFlex(ClimbConstants.MotorId, MotorType.kBrushless);
        m_encoder = m_motor.getAbsoluteEncoder();
        m_controller = m_motor.getClosedLoopController();
        m_motor.configure(ClimberConfig.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setAngle(double angle) {
        targetAngle = angle;
        m_controller.setReference(angle, ControlType.kPosition);
    }

    public void setVoltage(double volts) {
        m_controller.setReference(volts, ControlType.kVoltage);
    }

    public boolean isStored() {
        return m_encoder.getPosition() < 2;
    }

    public boolean onTarget() {
        return MathUtil.isNear(getAngle(), targetAngle, 3);
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Angle", m_encoder.getPosition());
    }
}
