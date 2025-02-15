// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import static frc.robot.util.Helpers.betterModulus;
import static frc.robot.util.Helpers.tan;
import static frc.robot.util.Helpers.tyToDistance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;

/** An example command that uses an example subsystem. */
public class AlignToTag extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_robotDrive;
  private final PIDController m_xController;
  private final PIDController m_yController;
  private final String limelightName = VisionConstants.ReefLightLightName;
  private double tolerance;
  private final PIDController m_turnPID;

  public enum Direction {
    LEFT, RIGHT
  }

  /**
   * Aligns robot to the reef for scoring
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToTag(DriveSubsystem subsystem) {
    m_robotDrive = subsystem;
    m_xController = new PIDController(DriveConstants.xTranslationkP, DriveConstants.xTranslationkI, DriveConstants.xTranslationkD);
    m_yController = new PIDController(DriveConstants.yTranslationkP, DriveConstants.yTranslationkI, DriveConstants.yTranslationkD);
    m_turnPID = new PIDController(DriveConstants.TurnkP, DriveConstants.TurnkI, DriveConstants.TurnkD);
  
    m_xController.setIZone(0.08); // 0.04
    m_yController.setIZone(0.08);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnPID.enableContinuousInput(0, 360);
    m_yController.setSetpoint(0.57);
    m_yController.setTolerance(0.0175);
    switch (m_robotDrive.scoringSide) {
      case LEFT:
        double leftDistance1 = 0.188;
        double leftDistance2 = 0.242;
        m_xController.setSetpoint((leftDistance1 + leftDistance2) / 2); // mid 0.2165 
        tolerance = Math.abs((leftDistance1 - leftDistance2) / 2);
        break;
      case RIGHT:
        double rightDistance1 = -0.1405;
        double rightDistance2 = -0.2;
        m_xController.setSetpoint((rightDistance1 + rightDistance2) / 2); // mid -0.1701 
        tolerance = Math.abs((rightDistance1 - rightDistance2));
        break;
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnOutput = m_turnPID.calculate(betterModulus(m_robotDrive.getHeading(), 360), m_robotDrive.getAngleToReef());
    if (LimelightHelpers.getTV(limelightName)) {
      double yDistanceFromTag = tyToDistance(limelightName);
      double xInput = yDistanceFromTag * tan(LimelightHelpers.getTX(limelightName)); // makes align to tag work when not against the wall? 
      double xOutput = m_xController.calculate(xInput);
      double yOutput = -m_yController.calculate(yDistanceFromTag);

      if (m_yController.atSetpoint()) {
        yOutput = 0;
        turnOutput = 0;
      }
      m_robotDrive.drive(Math.min(yOutput, 0.3), Math.min(xOutput, 0.3), turnOutput, false);

      boolean aligned = Math.abs(xInput - m_xController.getSetpoint()) < tolerance && m_yController.atSetpoint();
      m_robotDrive.setAlignedToReef(aligned);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.drive(0,0,0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
