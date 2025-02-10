package frc.robot.commands.CoralIntake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Claw;

public class PositionCoral extends SequentialCommandGroup {
    
    public PositionCoral(Claw claw) {
        Trigger coralStored = new Trigger(claw::coralStored);
            addCommands(
                Commands.runOnce(() -> claw.runVoltage(1)),
                Commands.waitUntil(coralStored.negate()),
                Commands.runOnce(() -> claw.runVoltage(-2)),
                Commands.waitUntil(coralStored),
                Commands.runOnce(() -> claw.runVoltage(0))
            );
    }
}
