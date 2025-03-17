package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Arm.ArmConstants;
import frc.robot.Claw.ClawConstants.WristConstants;
import frc.robot.CoralMaster.CoralMaster;
import frc.robot.Elevator.ElevatorConstants;
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
