// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.DriveSubsystem;

import org.opencv.core.Point;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class PointAtAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private PIDController turnPID = new PIDController(0.02, 0, 0.0012);
  private double error, targetAngle;
  private CommandXboxController driverController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PointAtAngle(DriveSubsystem subsystem, double targetvalue, CommandXboxController controller) {
    m_subsystem = subsystem;
    targetAngle = targetvalue;
    driverController = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnPID.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = turnPID.calculate(betterModulus(m_subsystem.getHeading(), 360), targetAngle);
    m_subsystem.driveWithController(output, isScheduled());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double betterModulus(double x, double y) {
    return (x % y + y) % y;
  }
}
