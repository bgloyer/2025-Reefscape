package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClawConstants;

public class Claw extends SubsystemBase {
    
    private SparkMax m_wristMotor;
    private SparkMax m_intakeMotor;
    private SparkClosedLoopController m_wristController;

    public Claw() {
        m_intakeMotor = new SparkMax(ClawConstants.WristId, MotorType.kBrushless);
        m_wristMotor = new SparkMax(ClawConstants.IntakeId, MotorType.kBrushless);
        m_wristController = m_wristMotor.getClosedLoopController();
    }

    public void runIntake() {
        m_intakeMotor.setVoltage(ClawConstants.IntakeVoltage);
    }

    public void runOuttake() {
        m_intakeMotor.setVoltage(ClawConstants.OuttakeVoltage);
    }

    public void stopIntake() {
        m_intakeMotor.stopMotor();
    }

    public void setTargetAngle(double angle) {
        m_wristController.setReference(angle, ControlType.kPosition);
    }
}
