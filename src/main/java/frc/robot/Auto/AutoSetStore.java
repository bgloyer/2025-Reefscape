package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Claw.ClawConstants.WristConstants;
import frc.robot.subsystems.CoralMaster.CoralMaster;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.util.Level;

public class AutoSetStore extends SequentialCommandGroup {
    
    
    
    public AutoSetStore(CoralMaster coralMaster) {
            addCommands(
                Commands.runOnce(() -> coralMaster.setCurrentLevel(Level.STORE)),
                Commands.runOnce(() -> coralMaster.getArm().setTargetAngle(ArmConstants.Store), coralMaster.getArm()),
                Commands.waitUntil(coralMaster::onTarget),
                Commands.runOnce(() -> coralMaster.setState(ElevatorConstants.Station, ArmConstants.Station, WristConstants.Station), coralMaster),
                Commands.waitUntil(coralMaster.getElevator()::almostAtStore)
            );
    }

    
}
