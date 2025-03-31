package frc.robot.subsystems.Flooral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
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
    private final RelativeEncoder m_topEncoder;
    private final RelativeEncoder m_sideEncoder;
    private final SparkClosedLoopController m_pivotController;
    private final SparkAbsoluteEncoder m_encoder;
    private final AnalogInput m_beamBreak;
    private double targetAngle;
    private boolean holdingCoral;
    
        public Flooral() {
            m_beamBreak = new AnalogInput(FlooralConstants.BeamBreakChannel);
            m_pivotMotor = new SparkFlex(FlooralConstants.PivotId, MotorType.kBrushless);
            m_pivotMotor.configure(Configs.FlooralConfig.pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            m_sideMotor = new SparkFlex(FlooralConstants.SideId, MotorType.kBrushless);
            m_topMotor = new SparkFlex(FlooralConstants.TopId, MotorType.kBrushless);
            m_topMotor.configure(Configs.FlooralConfig.topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_sideMotor.configure(Configs.FlooralConfig.sideConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            m_pivotController = m_pivotMotor.getClosedLoopController();
            m_encoder = m_pivotMotor.getAbsoluteEncoder();
            m_topEncoder = m_topMotor.getEncoder();
            m_sideEncoder = m_sideMotor.getEncoder();
            targetAngle = 0;
        }

        public void setVoltage(double sideVolts, double topVolts) {
            // m_sideMotor.setVoltage(sideVolts);
            // m_topMotor.setVoltage(topVolts);
            m_sideMotor.getClosedLoopController().setReference(sideVolts, ControlType.kVoltage);
            m_topMotor.getClosedLoopController().setReference(topVolts, ControlType.kVoltage);
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
            holdingCoral = false;
        }
    
        public boolean coralStored() {
            return m_beamBreak.getValue() > 2000;
        }
    
        public boolean onTarget() {
            return MathUtil.isNear(targetAngle, m_encoder.getPosition(), 2);
        }
    
        public Command intakeCoralSequence() {
            // return runOnce(() -> setIntake());
            return Commands.sequence(
                runOnce(() -> setIntake()),
                Commands.waitUntil(this::coralStored),
                Commands.waitSeconds(0.1)
                ).finallyDo(() -> {
                    holdCoral();
                    setAngle(FlooralConstants.CoralStore);
                });
        }
    
        public void holdCoral() {
            m_sideMotor.getClosedLoopController().setReference(m_sideEncoder.getPosition(), ControlType.kPosition);
            m_topMotor.getClosedLoopController().setReference(m_topEncoder.getPosition(), ControlType.kPosition);
        }
    
        @Override
        public void periodic() {
            SmartDashboard.putNumber("Flooral Angle", m_encoder.getPosition());
            SmartDashboard.putNumber("Side Motor Current", m_sideMotor.getOutputCurrent());
            SmartDashboard.putBoolean("Flooral stored", coralStored());
            SmartDashboard.putBoolean("Flooral Pivot at store", atStore());
            SmartDashboard.putNumber("Flooral break beam raw input", m_beamBreak.getValue());
        }
    
        public Command setStore() {
            return runOnce(() -> {
                    setVoltage(0, 0);
                    setAngle(FlooralConstants.StationAngle);
                });
        }
    
    public void setHoldingCoralState(boolean value) {
            holdingCoral = value;
    } 

    public boolean getHoldingCoralState() {
        return holdingCoral;
    }

    public boolean atStore() {
        return MathUtil.isNear(FlooralConstants.CoralStore, m_encoder.getPosition(), 5);
    }

}
