package frc.robot.commands.CoralMaster;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.CoralMaster;
import frc.robot.util.Level;

public class SetStore extends SequentialCommandGroup {
    
    

    public SetStore(CoralMaster coralMaster) {
            addCommands(
                Commands.runOnce(() -> coralMaster.setCurrentLevel(Level.STORE)),
                Commands.runOnce(() -> coralMaster.getArm().setTargetAngle(ArmConstants.Store), coralMaster.getArm()),
                Commands.runOnce(() -> coralMaster.getClaw().setTargetAngle(120), coralMaster.getArm()),
                Commands.waitUntil(coralMaster::onTarget),
                Commands.runOnce(() -> coralMaster.getElevator().setTarget(Level.STORE.elevatorHeight)),
                Commands.waitUntil(coralMaster::onTarget),
                Commands.runOnce(() -> coralMaster.getClaw().setTargetAngle(Level.STORE.wristAngle), coralMaster.getArm())
            );
    }

    
}
