package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClawConstants;

public class Claw extends SubsystemBase {
    
    private SparkMax m_wristMotor;
    private SparkMax m_intakeMotor;

    public Claw() {
        m_intakeMotor = new SparkMax(ClawConstants.WristId, MotorType.kBrushless);
        m_wristMotor = new SparkMax(ClawConstants.IntakeId, MotorType.kBrushless);
    }
}
