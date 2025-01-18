// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignToTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_robotDrive;
  private final PIDController m_pidController;
  private final String limelightName = "limelight-threeg";
  private int dir;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToTag(DriveSubsystem subsystem, int dir) {
    m_robotDrive = subsystem;
    this.dir = dir;
    m_pidController = new PIDController(DriveConstants.translationkP, DriveConstants.translationkI, DriveConstants.translationkD);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (dir == -1)
      m_pidController.setSetpoint(17.9);
    else
      m_pidController.setSetpoint(-14.9);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(LimelightHelpers.getTV(limelightName)) {
      double output = m_pidController.calculate(-LimelightHelpers.getTY(limelightName));
      m_robotDrive.driveSideWays(output);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.driveSideWays(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
