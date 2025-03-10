package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardManager extends SubsystemBase {
    
    public static boolean intakeCoralInTheWayOverride = false;
    public static boolean useManualScoring = false;
    private String zeroSelected;
    private final SendableChooser<String> zeroChooser;

    public DashboardManager() {
        zeroChooser = new SendableChooser<>();
        SmartDashboard.putData("Alex's next big idea",  zeroChooser);
        zeroChooser.addOption("Arm", "Arm");
        zeroChooser.addOption("Claw", "Claw");
        zeroChooser.addOption("Elevator", "Elevator");
        zeroChooser.addOption("Algae Intake", "Algae Intake");
        SmartDashboard.putBoolean("Intake Coral In the Way Override", intakeCoralInTheWayOverride);
        SmartDashboard.putBoolean("Use Manual Scoring", useManualScoring);
    }
    @Override
    public void periodic() {
        intakeCoralInTheWayOverride = SmartDashboard.getBoolean("Intake Coral In the Way", false);
    }

    public String getZeroSubsystem() {
        return zeroChooser.getSelected();
    }

    public boolean getUseManualScoring() {
        return SmartDashboard.getBoolean("Use Manual Scoring", false);
    }


}
