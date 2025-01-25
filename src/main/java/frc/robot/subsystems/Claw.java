package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClawConstants;

public class Claw extends SubsystemBase {
    
    private SparkFlex m_wristMotor;
    private SparkFlex m_intakeMotor;
    private SparkClosedLoopController m_wristController;
    private SparkClosedLoopController m_intakeController;

    public Claw() {
        m_intakeMotor = new SparkFlex(ClawConstants.WristId, MotorType.kBrushless);
        m_wristMotor = new SparkFlex(ClawConstants.IntakeId, MotorType.kBrushless);
        m_wristController = m_wristMotor.getClosedLoopController();
        m_intakeController = m_intakeMotor.getClosedLoopController();
    }

    public void runIntake() {
        setTargetVelocity(ClawConstants.IntakeVelocity);
    }

    public void runOuttake() {
        setTargetVelocity(ClawConstants.OuttakeVelocity);
    }

    public void stopIntake() {
        m_intakeMotor.stopMotor();
    }

    public void setTargetAngle(double angle) {
        m_wristController.setReference(angle, ControlType.kMAXMotionPositionControl);
    }

    private void setTargetVelocity(double velocity) {
        m_intakeController.setReference(velocity, ControlType.kVelocity);
    }
}
