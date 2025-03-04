package frc.robot.commands.CoralMaster;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.constants.ArmConstants;
import frc.robot.util.Level;

public final class CoralFactories {
    public static Command SetLevel(Level level, Trigger alignedToReef, RobotContainer container) {
        return Commands.sequence(
            Commands.runOnce(() -> container.getCoral().setCurrentLevel(level)),
            Commands.runOnce(() -> container.getArm().setTargetAngle(ArmConstants.Store), container.getArm()),
            Commands.waitUntil(container.getArm()::onTarget),
            Commands.runOnce(() -> container.getCoral().setState(level.withOffset().elevatorHeight, level.withOffset().wristAngle), container.getCoral()).onlyIf(RobotModeTriggers.teleop()),
            Commands.waitUntil(() -> container.getElevator().approachingHeight(level)),
            Commands.run(() -> container.getCoral().setState(level.withOffset()), container.getCoral()).until(alignedToReef.and(container.getCoral()::onTarget)),
            new Score(container.getCoral()).onlyIf(() -> level.isReefScoringPosition));
    }
}
