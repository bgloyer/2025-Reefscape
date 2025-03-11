package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeIntakeConstants;
import frc.robot.constants.Configs.AlgaeIntakeConfig;

public class AlgaeIntake extends TorSubsystemBase {
    
    private final SparkFlex m_pivotMotor; 
    private final SparkMax m_rollerMotor; 
    private final RelativeEncoder m_encoder; 
    private final SparkClosedLoopController m_pivotController; 
    private final SparkClosedLoopController m_rollerController; 
    
    public AlgaeIntake() {
        m_pivotMotor = new SparkFlex(AlgaeIntakeConstants.PivotMotorId, MotorType.kBrushless);
        m_encoder = m_pivotMotor.getEncoder();
        m_pivotController = m_pivotMotor.getClosedLoopController();
        m_pivotMotor.configure(AlgaeIntakeConfig.pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        setAngle(0);
        m_rollerMotor = new SparkMax(AlgaeIntakeConstants.RollerMotorId, MotorType.kBrushless);
        m_rollerController = m_rollerMotor.getClosedLoopController();
        m_rollerMotor.configure(AlgaeIntakeConfig.rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setAngle(double angle) {
        m_pivotController.setReference(angle, ControlType.kPosition);
        // m_pivotController.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, calcFeedForward());
    }

    public void setVoltage(double volts) {
        m_rollerController.setReference(volts, ControlType.kVoltage);
    }

    public Command runOuttake() {
        return startEnd(
            () -> setVoltage(AlgaeIntakeConstants.OuttakeVoltage), 
            () -> setVoltage(0));
    }

    private double calcFeedForward() {
        return AlgaeIntakeConstants.kG * Math.sin(Math.toRadians(m_encoder.getPosition()));
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Angle", m_encoder.getPosition());
    }

    public void stopPid() {
        m_pivotController.setReference(0, ControlType.kVoltage);
    }

    public void setZero() {
        m_encoder.setPosition(0);


        // in memory of james
        // m_pivotMotor.configure(AlgaeIntakeConfig.pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // setAngle(0);
    }

    @Override
    public void toggleIdleMode() {
        SparkBaseConfig config = super.getIdleModeConfig();
        m_pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public boolean isBrakeMode() {
        return m_pivotMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
    }
}
