package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.AlgaeIntakeConstants;

public final class ClimbFactories {
    
    public static Command readyClimb(RobotContainer container) {
        return Commands.sequence(
            Commands.runOnce(() -> container.getAlgaeIntake().setAngle(AlgaeIntakeConstants.ClimbReadyAngle)),
            Commands.runOnce(() -> container.getClaw().setTargetAngle(90)),
            Commands.runOnce(() -> container.getArm().setTargetAngle(90)),
            Commands.waitUntil(() ->container.getArm().onTarget()),
            Commands.runOnce(() -> container.getClimber().setAngle(ClimbConstants.ReadyAngle)),
            Commands.waitUntil(() -> container.getClimber().onTarget()),
            Commands.runOnce(() -> container.getClaw().setTargetAngle(0)),
            Commands.waitUntil(() -> container.getClaw().onTarget()),
            Commands.runOnce(() -> container.getArm().setTargetAngle(110)),
            Commands.waitUntil(() ->container.getArm().onTarget()));
            
      }
    

      public static Command storeClimb(RobotContainer container) {
        return Commands.sequence(
          Commands.runOnce(() -> container.getArm().setTargetAngle(90)),
          Commands.waitUntil(() ->container.getArm().onTarget()),
          Commands.runOnce(() -> container.getClaw().setTargetAngle(90)),
          Commands.waitUntil(() ->container.getClaw().onTarget()),
          Commands.runOnce(() -> container.getClimber().setAngle(ClimbConstants.StoreAngle)),
          Commands.waitUntil(() -> container.getClimber().onTarget()),
          Commands.runOnce(() -> container.getAlgaeIntake().setAngle(AlgaeIntakeConstants.StoreAngle)),
          Commands.runOnce(() -> container.getCoral().setStore()));
      }

      public static Command climb(RobotContainer container) {
        return Commands.sequence(
          Commands.runOnce(() -> container.getClimber().setAngle(ClimbConstants.ClimbAngle)),
          Commands.runOnce(() -> container.getAlgaeIntake().setAngle(AlgaeIntakeConstants.ClimbStoreAngle))
        );
      }
    
}
