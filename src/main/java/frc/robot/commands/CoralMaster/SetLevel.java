package frc.robot.commands.CoralMaster;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.CoralMaster;
import frc.robot.util.Level;

public class SetLevel extends SequentialCommandGroup {
    
    

    public SetLevel(Level level, CoralMaster coralMaster, CommandXboxController controller) {
            addCommands(
                Commands.runOnce(() -> coralMaster.setCurrentLevel(level)),
                Commands.runOnce(() -> coralMaster.getArm().setTargetAngle(ArmConstants.Store), coralMaster.getArm()),
                Commands.waitUntil(coralMaster::onTarget),
                Commands.runOnce(() -> coralMaster.setState(level.elevatorHeight, ArmConstants.Store, level.wristAngle), coralMaster),
                Commands.waitUntil(coralMaster::onTarget),
                Commands.runOnce(() -> coralMaster.getArm().setTargetAngle(level.armAngle), coralMaster.getArm()),
                Commands.waitUntil(coralMaster::onTarget),
                new Score(coralMaster, controller).onlyIf(() -> level.isReefScoringPosition)  
            );
    }

    
}
