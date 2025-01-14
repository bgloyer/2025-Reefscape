// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import org.opencv.core.Point;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PointAtAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private PIDController turnPID = new PIDController(0.02, 0, 0.0012);
  private double targetAngle;
  private DoubleSupplier m_angleSupplier;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PointAtAngle(DriveSubsystem subsystem, double targetvalue) {
    m_subsystem = subsystem;
    targetAngle = targetvalue;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public PointAtAngle(DriveSubsystem subsystem, DoubleSupplier supplier) {
    m_subsystem = subsystem;
    m_angleSupplier = supplier;
    targetAngle = supplier.getAsDouble();
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

  private double betterModulus(double x, double y) {
    return (x % y + y) % y;
  }
}
