package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.ClimbConstants;

public final class ClimbFactories {
    
    public static Command readyClimb(RobotContainer container) {
        return Commands.sequence(
            Commands.runOnce(() -> container.getClaw().setTargetAngle(90)),
            Commands.runOnce(() -> container.getArm().setTargetAngle(90)),
            Commands.waitUntil(() ->container.getArm().onTarget()),
            Commands.runOnce(() -> container.getClimber().setAngle(ClimbConstants.ReadyAngle)),
            Commands.waitUntil(() -> container.getClimber().onTarget()),
            Commands.runOnce(() -> container.getClaw().setTargetAngle(0)),
            Commands.waitUntil(() -> container.getClaw().onTarget()),
            Commands.runOnce(() -> container.getArm().setTargetAngle(110)),
            Commands.waitUntil(() ->container.getArm().onTarget()));
            
            // Commands.runOnce(() -> container.getAlgaeIntake().setAngle(AlgaeIntakeConstants.ClimbAngle)),
            // new WaitCommand(0.8));
      }
    

      public static Command storeClimb(RobotContainer container) {
        return Commands.sequence(
          Commands.runOnce(() -> container.getArm().setTargetAngle(90)),
          Commands.waitUntil(() ->container.getArm().onTarget()),
          Commands.runOnce(() -> container.getClaw().setTargetAngle(90)),
          Commands.waitUntil(() ->container.getClaw().onTarget()),
          Commands.runOnce(() -> container.getClimber().setAngle(ClimbConstants.StoreAngle)),
          Commands.waitUntil(() -> container.getClimber().onTarget()),
          Commands.runOnce(() -> container.getCoral().setStore()));
      }
    
}
