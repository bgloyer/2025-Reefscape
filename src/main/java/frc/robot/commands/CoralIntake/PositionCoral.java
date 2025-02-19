package frc.robot.commands.CoralIntake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Claw;

public class PositionCoral extends SequentialCommandGroup {
    
    public PositionCoral(Claw claw) {
        Trigger coralStored = new Trigger(claw::backLaserTriggered);
        Trigger frontCoralTriggered = new Trigger(claw::frontLaserTriggered);
            addCommands(
                Commands.runOnce(() -> claw.runVoltage(1)),
                Commands.waitUntil(frontCoralTriggered.negate().or(coralStored.negate())),
                Commands.runOnce(() -> claw.runVoltage(-1.5)),
                Commands.waitUntil(coralStored),
                Commands.runOnce(() -> claw.runVoltage(0))
            );
    }
}
