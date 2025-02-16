// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralMaster;

import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CoralMaster;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SpacedIntakeCoral extends Command {
  private CoralMaster m_subsystem;
  private PIDController turnPID = new PIDController(DriveConstants.TurnkP, DriveConstants.TurnkI, DriveConstants.TurnkD);
  private double targetAngle;
  private DriveSubsystem m_robotDrive;
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  
    /**
     * Creates a new IntakeCoral command
     * 
     *
     * @param subsystem The subsystem used by this command.
     */
    public SpacedIntakeCoral(CoralMaster subsystem, DriveSubsystem robotDrive) {
      m_subsystem = subsystem;
      m_robotDrive = robotDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.setAlignedToReef(false);
    m_subsystem.setSpacedIntake();
    targetAngle = m_robotDrive.getStationAngle();
    turnPID.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = turnPID.calculate(betterModulus(m_robotDrive.getHeading(), 360), targetAngle);
    m_robotDrive.driveWithController(output, true);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopIntake();
    // m_subsystem.setStore();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.coralStored();
  }

  private double betterModulus(double x, double y) {
    return ((x % y + y) % y);
  }
}
