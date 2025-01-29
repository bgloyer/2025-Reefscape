package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CoralMaster.Score;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ClawConstants.WristConstants;

public class CoralMaster extends SubsystemBase {
    private Arm m_arm;
    private Elevator m_elevator;
    private Claw m_claw;
    private LaserCan m_laser;
    public CoralMaster(Arm arm, Elevator elevator, Claw claw ){
        m_arm = arm;
        m_elevator = elevator;
        m_claw = claw;
        m_laser = new LaserCan(Constants.CoralLaserCanID);
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

    public boolean coralStored() {
        return (m_laser.getMeasurement() != null) && (m_laser.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) ;
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

    public double getDistance() {
        if (m_laser.getMeasurement() == null)
            return 0;
        else
            return m_laser.getMeasurement().distance_mm;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Laser Can Measurement", getDistance());
        SmartDashboard.putBoolean("Game Piece stored", coralStored());
    }
}
