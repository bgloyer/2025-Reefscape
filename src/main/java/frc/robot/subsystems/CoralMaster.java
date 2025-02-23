package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ClawConstants.WristConstants;
import frc.robot.util.Level;

public class CoralMaster extends SubsystemBase {
    private final Arm m_arm;
    private final Elevator m_elevator;
    private final Claw m_claw;
    private Level level = Level.STORE;
    private boolean useIntakeAutoAlign = true;
    
    public CoralMaster(Arm arm, Elevator elevator, Claw claw ) {
        m_arm = arm;
        m_elevator = elevator;
        m_claw = claw;
    }

    public Arm getArm() {
        return m_arm;
    }

    public Claw getClaw() {
        return m_claw;
    }

    public Elevator getElevator() {
        return m_elevator;
    }

    public void setState(double elevatorPosition, double armAngle, double clawangle) {
        m_elevator.setTarget(elevatorPosition);
        m_arm.setTargetAngle(armAngle);
        m_claw.setTargetAngle(clawangle);
    }

    public void setState(double elevatorPosition, double clawangle) {
        m_elevator.setTarget(elevatorPosition);
        m_claw.setTargetAngle(clawangle);
    }

    public void setState(Level level) {
        m_elevator.setTarget(level.elevatorHeight);
        m_arm.setTargetAngle(level.armAngle);
        m_claw.setTargetAngle(level.wristAngle);
    }

    public void setIntake() {
        setState(ElevatorConstants.Station, ArmConstants.Station, WristConstants.Station); 
        m_claw.runIntake();  
    }

    public void setSpacedIntake() {
        m_claw.runIntake();  
        setState(0, 35.77, 25.87); 
    }

    public void setStore() {
        setState(ElevatorConstants.Store, ArmConstants.Store, WristConstants.Store);
    }

    public void runIntake() {
        m_claw.runIntake();
    }

    public void runOuttake() {
        m_claw.runOuttake();
    }

    public void stopIntake() {
        m_claw.stopIntake();
    }

    public boolean onTarget() {
        return m_arm.onTarget() && m_claw.onTarget() && m_elevator.onTarget();
    }

    public boolean coralStored() {
        return m_claw.backLaserTriggered();
    }

    public boolean readyToScore() {
        return onTarget() && level.isReefScoringPosition;
    }

    public void setCurrentLevel(Level level) {
        this.level = level;
    }

    public boolean scoringButNotDealg() {
        return level == Level.FOUR || level == Level.TWO || level == Level.THREE || level == Level.ONE;
    }

    public Level getLevel() {
        return level;
    }

    public boolean useIntakeAutoAlign() {
        return useIntakeAutoAlign;
    }

    public Command toggleIntakeAutoAlign() {
        return runOnce(() -> {
            useIntakeAutoAlign = !useIntakeAutoAlign;
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Enable Intake Auto Align", useIntakeAutoAlign);
    }
}
