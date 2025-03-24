package frc.robot.subsystems.CoralMaster;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Drive.DriveAutomation.AlignToReef.Direction;
import frc.robot.util.Level;

public final class CoralFactories {

    // public static Command TopAlgaeGrab(RobotContainer container) {
    //     return Commands.sequence(
    //       Commands.runOnce(() -> container.getCoral().setranL4(container.getCoral().coralStored())),
    //       new SetLevel(Level.FOUR, container.getCoral(), container.alignedToReef).until(container.coralStored.negate()),
    //       Commands.runOnce(() -> container.getDrive().setScoringSide(Direction.MIDDLE)),
    //       Commands.waitUntil(alignedToMiddle.or(container.getCoral()::getranL4)),
    //       Commands.parallel(
    //         Commands.runOnce(() -> m_claw.runVoltage(7)),
    //         new SetLevel(Level.BOTTOMALGAEGRAB, container.getCoral(), m_driverController, alignedToReef)));

    // }
}
