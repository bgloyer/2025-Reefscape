package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardManager extends SubsystemBase {
    
    public static boolean intakeCoralInTheWay = false;
    private String zeroSelected;
    private final SendableChooser<String> zeroChooser;

    public DashboardManager() {
        zeroChooser = new SendableChooser<>();
        SmartDashboard.putData("Alex's next big idea",  zeroChooser);
        zeroChooser.addOption("Arm", "Arm");
        zeroChooser.addOption("Claw", "Claw");
        zeroChooser.addOption("Elevator", "Elevator");
        zeroChooser.addOption("Algae Intake", "Algae Intake");
        SmartDashboard.putBoolean("Intake Coral In the Way", false);
    }
    @Override
    public void periodic() {
        intakeCoralInTheWay = SmartDashboard.getBoolean("Intake Coral In the Way", false);
    }

    public String getZeroSubsystem() {
        return zeroChooser.getSelected();
    }
    public void zeroStuff() {

    }
}
