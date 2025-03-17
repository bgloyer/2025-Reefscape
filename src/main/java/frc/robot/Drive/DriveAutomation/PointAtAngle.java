// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive.DriveAutomation;

import frc.robot.Drive.DriveConstants;
import frc.robot.Drive.DriveSubsystem;
import frc.robot.constants.AutoConstants;

import static frc.robot.util.Helpers.betterModulus;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;

public class PointAtAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  // private PIDController turnPID = new PIDController(DriveConstants.TurnkP, DriveConstants.TurnkI, DriveConstants.TurnkD);
  private ProfiledPIDController turnPID = new ProfiledPIDController(DriveConstants.TurnkP, DriveConstants.TurnkI, DriveConstants.TurnkD, new Constraints(DriveConstants.TurnMaxVelocity, DriveConstants.TurnMaxAccel));
  private double targetAngle;

  /**
   * 
   *
   * @param subsystem The subsystem used by this command.
   * @param angle direction on field
   */
  public PointAtAngle(DriveSubsystem subsystem, double angle) {
    m_subsystem = subsystem;
    targetAngle = angle;
    turnPID.enableContinuousInput(0, 360);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnPID.reset(m_subsystem.getHeading());
    turnPID.setGoal(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = turnPID.calculate(betterModulus(m_subsystem.getHeading(), 360));
    m_subsystem.driveWithController(output, true);
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
