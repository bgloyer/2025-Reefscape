package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardManager extends SubsystemBase {
    
    public static boolean intakeCoralInTheWay = false;

    public DashboardManager() {
        SmartDashboard.putBoolean("Intake Coral In the Way", false);
    }
    @Override
    public void periodic() {
        intakeCoralInTheWay = SmartDashboard.getBoolean("Intake Coral In the Way", false);
    }
}
