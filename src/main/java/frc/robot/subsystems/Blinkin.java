package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.BlinkinConstants;

public class Blinkin extends SubsystemBase{

    private final Spark m_Blinkin;
    public Blinkin() {
        m_Blinkin = new Spark(BlinkinConstants.BlinkinPwmPort);
    }

    public void setColorNotCommand(double value) {
        m_Blinkin.set(value);
    }

    public Command setColor(double value) {
        return runOnce(() -> m_Blinkin.set(value));
    }

    public void setRandom() {
        setColorNotCommand(Math.round((Math.random() * 100.0)) / 100.0);
    }
}
