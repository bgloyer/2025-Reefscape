package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.ClawConstants.WristConstants;
import frc.robot.subsystems.CoralMaster;
import frc.robot.util.Level;

public class AutoSetStore extends SequentialCommandGroup {
    
    
    
    public AutoSetStore(CoralMaster coralMaster) {
            addCommands(
                Commands.runOnce(() -> coralMaster.setCurrentLevel(Level.STORE)),
                Commands.runOnce(() -> coralMaster.getArm().setTargetAngle(ArmConstants.Station), coralMaster.getArm()),
                Commands.waitUntil(coralMaster::onTarget),
                Commands.runOnce(() -> coralMaster.setState(ElevatorConstants.Station, ArmConstants.Station, WristConstants.Station), coralMaster),
                Commands.waitUntil(coralMaster.getElevator()::almostAtStore)
            );
    }

    
}
