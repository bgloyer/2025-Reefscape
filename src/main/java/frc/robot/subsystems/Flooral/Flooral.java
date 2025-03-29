package frc.robot.subsystems.Flooral;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Configs;
import frc.robot.util.Helpers;

public class Flooral extends SubsystemBase {
    private final SparkFlex m_pivotMotor;
    private final SparkFlex m_topMotor;
    private final SparkFlex m_sideMotor;
    private final SparkClosedLoopController m_pivotController;
    private final SparkAbsoluteEncoder m_encoder;
    private final DigitalInput m_beamBreak;
    private double targetAngle;

    public Flooral() {
        m_beamBreak = new DigitalInput(FlooralConstants.BeamBreakChannel);
        m_pivotMotor = new SparkFlex(FlooralConstants.PivotId, MotorType.kBrushless);
        m_pivotMotor.configure(Configs.FlooralConfig.pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_sideMotor = new SparkFlex(FlooralConstants.SideId, MotorType.kBrushless);
        m_topMotor = new SparkFlex(FlooralConstants.TopId, MotorType.kBrushless);
        m_topMotor.configure(Configs.FlooralConfig.topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_sideMotor.configure(Configs.FlooralConfig.sideConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_pivotController = m_pivotMotor.getClosedLoopController();
        m_encoder = m_pivotMotor.getAbsoluteEncoder();
        targetAngle = 0;
    }

    public void setVoltage(double sideVolts, double topVolts) {
        m_sideMotor.setVoltage(sideVolts);
        m_topMotor.setVoltage(topVolts);
    }

    public Command stopMotor() {
        return runOnce(() -> setVoltage(0, 0));
    }

    public void setAngle(double angle) {
        targetAngle = angle;
        m_pivotController.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, calcFeedForward());
    }

    private double calcFeedForward() {
        return FlooralConstants.kG * Helpers.sin(m_encoder.getPosition() - FlooralConstants.HardStopOffset);
    }

    public void setIntake() {
        setVoltage(FlooralConstants.SideVoltage, FlooralConstants.TopVoltage);
        setAngle(FlooralConstants.IntakeAngle); 
    }

    public boolean coralStored() {
        return m_beamBreak.get();
    }

    public boolean onTarget() {
        return MathUtil.isNear(targetAngle, m_encoder.getPosition(), 5);
    }

    public Command intakeCoralSequence() {
        return runOnce(() -> setIntake());
        // return Commands.sequence(
            // runOnce(() -> setIntake()),
            // Commands.waitUntil(this::coralStored),
            // runOnce(() -> {
            //     stopMotor();
            //     setAngle(FlooralConstants.CoralStore);
            // }));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flooral Angle", m_encoder.getPosition());
        SmartDashboard.putNumber("Side Motor Current", m_sideMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Flooral stored", coralStored());
    }
}
