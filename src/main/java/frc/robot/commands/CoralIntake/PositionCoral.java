package frc.robot.commands.CoralIntake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Claw;

public class PositionCoral extends SequentialCommandGroup{
    
    public PositionCoral(Claw claw) {
        addCommands(
            Commands.runOnce(() -> claw.runVoltage(0.5)),
            Commands.waitUntil(new Trigger(claw::coralStored).negate()),
            Commands.runOnce(() -> claw.runVoltage(-0.5)),
            Commands.waitUntil(claw::coralStored)
        );
    }
}
