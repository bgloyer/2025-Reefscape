package frc.robot.subsystems.Flooral;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Configs;
import frc.robot.util.Helpers;

public class Flooral extends SubsystemBase {
    private final SparkFlex m_pivotMotor;
    private final SparkFlex m_topMotor;
    private final SparkFlex m_sideMotor;
    private final SparkClosedLoopController m_pivotController;
    private final SparkAbsoluteEncoder m_encoder;

    public Flooral() {
        m_pivotMotor = new SparkFlex(FlooralConstants.PivotId, MotorType.kBrushless);
        m_pivotMotor.configure(Configs.FlooralConfig.pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_sideMotor = new SparkFlex(FlooralConstants.SideId, MotorType.kBrushless);
        m_topMotor = new SparkFlex(FlooralConstants.TopId, MotorType.kBrushless);
        m_pivotController = m_pivotMotor.getClosedLoopController();
        m_encoder = m_pivotMotor.getAbsoluteEncoder();

    }

    public void setVoltage(double volts) {
        m_sideMotor.setVoltage(volts);
        m_topMotor.setVoltage(volts * FlooralConstants.SideToTopMotorRatio);
    }

    public void setAngle(double angle) {
        m_pivotController.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, calcFeedForward());
    }

    private double calcFeedForward() {
        return FlooralConstants.kG * Helpers.sin(m_encoder.getPosition());
    }
}
