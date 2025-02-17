// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import static frc.robot.util.Helpers.betterModulus;
import static frc.robot.util.Helpers.tan;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CoralMaster;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Helpers;
import frc.robot.util.LimelightHelpers;

/** An example command that uses an example subsystem. */
public class AutoAlignToStationTag extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_robotDrive;
  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final String limelightName = VisionConstants.ElevatorLimelightName;
  private double tolerance;
  private final ProfiledPIDController m_turnPID;
  private final CoralMaster m_coralMaster;

  public enum Direction {
    LEFT, RIGHT
  }

  /**
   * Aligns robot to the reef for scoring
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlignToStationTag(DriveSubsystem subsystem, CoralMaster coralMaster) {
    m_robotDrive = subsystem;
    m_xController = new ProfiledPIDController(DriveConstants.xTranslationkP, DriveConstants.xTranslationkI, DriveConstants.xTranslationkD, new Constraints(DriveConstants.xTranslationMaxVel, DriveConstants.xTranslationMaxAccel));
    m_yController = new ProfiledPIDController(DriveConstants.yTranslationkP, DriveConstants.yTranslationkI, DriveConstants.yTranslationkD, new Constraints(DriveConstants.yTranslationMaxVel, DriveConstants.yTranslationMaxAccel));
    m_turnPID = new ProfiledPIDController(DriveConstants.TurnkP, DriveConstants.TurnkI, DriveConstants.TurnkD, new Constraints(DriveConstants.TurnMaxVelocity, DriveConstants.TurnMaxAccel));
    m_xController.setIZone(0.08); 
    m_yController.setIZone(0.08);
    m_turnPID.enableContinuousInput(0, 360);
    m_coralMaster = coralMaster;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, coralMaster);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralMaster.setIntake();
    m_turnPID.reset(m_robotDrive.getHeading());
    m_yController.setGoal(0.6127);
    m_yController.setTolerance(0.0175);
    m_xController.setGoal(-0.1967); // mid 0.2165 
    m_turnPID.setGoal(m_robotDrive.getStationAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnOutput = m_turnPID.calculate(betterModulus(m_robotDrive.getHeading(), 360));
    if (LimelightHelpers.getTV(limelightName)) {
      double yDistanceFromTag = Helpers.tyToDistance(limelightName);
      double xInput = yDistanceFromTag * tan(-LimelightHelpers.getTX(limelightName)); 
      double xOutput = m_xController.calculate(xInput);
      double yOutput = -m_yController.calculate(yDistanceFromTag);

      if (m_yController.atGoal()) {
        yOutput = 0;
        turnOutput = 0;
      }
      m_robotDrive.drive(Math.min(yOutput, 0.3), Math.min(xOutput, 0.3), turnOutput, false);

      boolean aligned = m_xController.atGoal() && m_yController.atGoal();
      m_robotDrive.setAlignedToReef(aligned);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralMaster.stopIntake();
    m_robotDrive.drive(0,0,0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_coralMaster.coralStored();
  }
}
