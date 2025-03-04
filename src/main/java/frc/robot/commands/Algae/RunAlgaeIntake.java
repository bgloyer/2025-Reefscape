// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Algae;

import frc.robot.constants.AlgaeIntakeConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunAlgaeIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AlgaeIntake m_intake;
  private double volts;
  private double angle;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunAlgaeIntake(double angle, double volts, AlgaeIntake subsystem) {
    m_intake = subsystem;
    this.volts = volts;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setAngle(AlgaeIntakeConstants.IntakeAngle);
    m_intake.setVoltage(volts);
    m_intake.setAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setAngle(AlgaeIntakeConstants.StoreAngle);
    m_intake.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
