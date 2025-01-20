package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase {
    private SparkFlex m_motor;

    public Arm() {
        m_motor = new SparkFlex(ArmConstants.MotorId, null);
    }
}
