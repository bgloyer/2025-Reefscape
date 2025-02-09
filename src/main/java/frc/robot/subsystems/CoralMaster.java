package frc.robot.subsystems;

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

    public void setState(double elevatorPosition, double armAngle, double clawangle) {
        m_elevator.setTarget(elevatorPosition);
        m_arm.setTargetAngle(armAngle);
        m_claw.setTargetAngle(clawangle);
    }

    public void setIntake() {
        setState(ElevatorConstants.Station, ArmConstants.Station, WristConstants.Station); 
        m_claw.runIntake();  
    }

    public void setStore() {
        setState(ElevatorConstants.Store, ArmConstants.Store, WristConstants.Store);
    }

    public void setL1() {
        setState(ElevatorConstants.L1, ArmConstants.L1, WristConstants.L1);
    }

    public void setL2() {
        setState(ElevatorConstants.L2, ArmConstants.L2, WristConstants.L2);
    }

    public void setL3() {
        setState(ElevatorConstants.L3, ArmConstants.L3, WristConstants.L3);
    }

    public void setL4() {
        setState(ElevatorConstants.L4, ArmConstants.L4, WristConstants.L4);
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
        return m_claw.coralStored();
    }

    public boolean readyToScore() {
        return onTarget() && level.isReefScoringPosition;
    }

    public void setCurrentLevel(Level level) {
        this.level = level;
    }

    public boolean isLevelFour() {
        return level == Level.FOUR;
    }

    @Override
    public void periodic() {
    }
}
