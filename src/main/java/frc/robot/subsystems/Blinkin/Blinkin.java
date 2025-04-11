package frc.robot.subsystems.Blinkin;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase{

    private final Spark m_Blinkin;
    public Blinkin() {
        m_Blinkin = new Spark(BlinkinConstants.BlinkinPwmPort);
    }

    public void setColor(double value) {
        m_Blinkin.set(value);
    }
}
