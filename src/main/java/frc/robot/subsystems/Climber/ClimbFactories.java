package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public final class ClimbFactories {
    
    public static Command readyClimb(RobotContainer container) {
        return Commands.sequence(
          Commands.runOnce(() -> container.getClaw().setTargetAngle(90)),
          Commands.runOnce(() -> container.getArm().setTargetAngle(-98), container.getArm()),
          Commands.waitUntil(() ->container.getArm().onTarget()),
          Commands.runOnce(() -> container.getClimber().setAngle(ClimbConstants.ReadyAngle)),
          Commands.waitUntil(() -> container.getClimber().onTarget()),
          Commands.runOnce(() -> container.getFlooral().setAngle(0)),
          Commands.runOnce(() -> container.getClaw().setTargetAngle(178)),
          Commands.waitUntil(() -> container.getClaw().onTarget()),
          Commands.runOnce(() -> container.getArm().setTargetAngle(-103)),
          Commands.waitUntil(() ->container.getArm().onTarget()));
            
      }
    

      public static Command storeClimb(RobotContainer container) {
        return Commands.sequence(
          container.getFlooral().setStore(),
          Commands.runOnce(() -> container.getClaw().setTargetAngle(178)),
          Commands.runOnce(() -> container.getArm().setTargetAngle(-98)),
          Commands.waitUntil(() ->container.getArm().onTarget()),
          Commands.runOnce(() -> container.getClaw().setTargetAngle(90)),
          Commands.waitUntil(() ->container.getClaw().onTarget()),
          Commands.runOnce(() -> container.getClimber().setAngle(ClimbConstants.StoreAngle)),
          Commands.waitUntil(() -> container.getClimber().onTarget()),
          Commands.runOnce(() -> container.getCoral().setStore()));
      }

      public static Command climb(RobotContainer container) {
        return Commands.sequence(
          Commands.runOnce(() -> container.getClimber().setAngle(ClimbConstants.ClimbAngle))
        );
      }
    
}
