package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TorSubsystemBase extends SubsystemBase {

    public abstract void toggleIdleMode();
    public abstract boolean isBrakeMode();
    public abstract void setZero();

    public SparkBaseConfig getIdleModeConfig() {
         SparkFlexConfig config = new SparkFlexConfig();
            if(isBrakeMode()) {
                return config.idleMode(IdleMode.kCoast);
            }
            else {
                return config.idleMode(IdleMode.kBrake);
            }
    }
}
