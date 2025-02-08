// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignToTag extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_robotDrive;
  private final PIDController m_pidController;
  private final String limelightName = VisionConstants.LightLightName;
  private Direction dir;

  public enum Direction {
    LEFT, RIGHT
  }

  /**
   * Aligns robot to the reef for scoring
   *
   * @param subsystem The subsystem used by this command.
   * @param dir       the left or right section of the reef
   */
  public AlignToTag(DriveSubsystem subsystem, Direction dir) {
    m_robotDrive = subsystem;
    this.dir = dir;
    m_pidController = new PIDController(DriveConstants.TranslationkP, DriveConstants.TranslationkI,
        DriveConstants.TranslationkD);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (dir) {
      case LEFT:
        m_pidController.setSetpoint(19.875); //L Side: 21.62-18.13
        m_pidController.setTolerance(1.75);
        break;
      case RIGHT:
        m_pidController.setSetpoint(-14.75); //R Side: 12.7-16.8 
        m_pidController.setTolerance(2.05);
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(limelightName)) {
      double output = m_pidController.calculate(LimelightHelpers.getTX(limelightName));
      m_robotDrive.driveSideways(output);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.driveSideways(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }
}
