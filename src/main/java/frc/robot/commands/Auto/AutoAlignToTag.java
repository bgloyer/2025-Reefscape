// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.util.Helpers.*;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoAlignToTag extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_robotDrive;
  private final PIDController m_pidController;
  private final String limelightName = VisionConstants.LightLightName;
  private double tolerance;
  public enum Direction {
    LEFT, RIGHT
  }

  /**
   * Aligns robot to the reef for scoring
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlignToTag(DriveSubsystem subsystem) {
    m_robotDrive = subsystem;
    m_pidController = new PIDController(DriveConstants.xTranslationkP, DriveConstants.xTranslationkI,
        DriveConstants.xTranslationkD);
    m_pidController.setIZone(0.04);
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_robotDrive.scoringSide) {
      case LEFT:
        double leftDistance1 = 0.188;
        double leftDistance2 = 0.242;
        m_pidController.setSetpoint((leftDistance1 + leftDistance2) / 2); //L Side: 0.242  0.188 Left mid 0.21 R mid -0.17
        tolerance = Math.abs((leftDistance1 - leftDistance2) / 2);
        break;
      case RIGHT:
        double rightDistance1 = -0.1405;
        double rightDistance2 = -0.2;
        m_pidController.setSetpoint((rightDistance1 + rightDistance2) / 2); //R Side: -0.1405   mid -0.1601  -0.20
        tolerance = Math.abs((rightDistance1 - rightDistance2));
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(limelightName)) {
      double input = tyToDistance(limelightName) * tan(LimelightHelpers.getTX(limelightName)); // makes align to tag work when not against the wall? 

      double output = m_pidController.calculate(input);
      // m_robotDrive.driveXY(output);
      m_robotDrive.setAlignedToReef(Math.abs(input - m_pidController.getSetpoint()) < tolerance);
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_robotDrive.driveSideways(0);
    m_robotDrive.setAlignedToReef(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_robotDrive.alignedToReef();
  }
}
