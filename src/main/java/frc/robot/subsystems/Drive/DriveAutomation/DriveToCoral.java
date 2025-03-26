// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive.DriveAutomation;

import static frc.robot.util.Helpers.tyToDistance;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Drive.DriveConstants;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.VisionConstants;
import frc.robot.util.Helpers;
import frc.robot.util.LimelightHelpers;

/** An example command that uses an example subsystem. */
public class DriveToCoral extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_robotDrive;
  private PIDController distancePid = new PIDController(0.1, 0, 0);
  private PIDController turnPid = new PIDController(DriveConstants.TurnkP, DriveConstants.TurnkI, DriveConstants.TurnkD);
  private double offset;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToCoral(DriveSubsystem subsystem) {
    m_robotDrive = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distancePid.setSetpoint(DriveConstants.CoralGroundIntakeDistance);
    turnPid.setSetpoint(0);
    offset = m_robotDrive.getHeading() - LimelightHelpers.getTX(VisionConstants.ElevatorLimelightName);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(VisionConstants.ElevatorLimelightName)) {
      offset = m_robotDrive.getHeading() - LimelightHelpers.getTX(VisionConstants.ElevatorLimelightName);
      double distToCoral = 1 / Helpers.tan(28 - LimelightHelpers.getTY(VisionConstants.ElevatorLimelightName));
      double driveOutput = -distancePid.calculate(distToCoral);
      m_robotDrive.drive(MathUtil.clamp(-0.1, driveOutput, 0.1),0, turnPid.calculate(LimelightHelpers.getTX(VisionConstants.ElevatorLimelightName)), false);
    } else {
      m_robotDrive.drive(0, 0, turnPid.calculate(m_robotDrive.getHeading() - offset), false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
