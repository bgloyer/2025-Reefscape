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
public class AutoAlignToTag extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_robotDrive;
  private final PIDController m_pidController;
  private final String limelightName = VisionConstants.LightLightName;
  private Direction dir;
  private double tolerance;
  public enum Direction {
    LEFT, RIGHT
  }

  /**
   * Aligns robot to the reef for scoring
   *
   * @param subsystem The subsystem used by this command.
   * @param dir       the left or right section of the reef
   */
  public AutoAlignToTag(DriveSubsystem subsystem) {
    m_robotDrive = subsystem;
    m_pidController = new PIDController(DriveConstants.TranslationkP, DriveConstants.TranslationkI,
        DriveConstants.TranslationkD);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_robotDrive.scoringSide) {
      case LEFT:
        double leftTx1 = 17.2;
        double leftTx2 = 21.65;
        m_pidController.setSetpoint((leftTx1 + leftTx2) / 2); 
        tolerance = Math.abs((leftTx1 - leftTx2) / 2);
        break;
      case RIGHT:
        double rightTx1 = -13.1;
        double rightTx2 = -18.98;
        m_pidController.setSetpoint((rightTx1 + rightTx2) / 2); //R Side: 12.7-16.8  -13.1-18.98
        tolerance = Math.abs((rightTx1 - rightTx2));
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

    m_robotDrive.setAlignedToReef(Math.abs(LimelightHelpers.getTX(limelightName) - m_pidController.getSetpoint()) < tolerance);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.driveSideways(0);
    m_robotDrive.setAlignedToReef(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_robotDrive.alignedToReef();
  }
}
