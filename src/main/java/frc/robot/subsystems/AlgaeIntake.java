package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeIntakeConstants;
import frc.robot.constants.Configs.AlgaeIntakeConfig;

public class AlgaeIntake extends SubsystemBase {
    
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

        m_rollerMotor = new SparkMax(AlgaeIntakeConstants.RollerMotorId, MotorType.kBrushless);
        m_rollerController = m_rollerMotor.getClosedLoopController();
        m_rollerMotor.configure(AlgaeIntakeConfig.rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setAngle(double angle) {
        m_pivotController.setReference(angle, ControlType.kPosition);
    }

    public void setVoltage(double volts) {
        m_rollerController.setReference(volts, ControlType.kVoltage);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Angle", m_encoder.getPosition());
    }
}
