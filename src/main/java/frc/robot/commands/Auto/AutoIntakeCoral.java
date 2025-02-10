// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.commands.CoralIntake.PositionCoral;
import frc.robot.subsystems.CoralMaster;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoIntakeCoral extends Command {
  private CoralMaster m_subsystem;
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  
    /**
     * Creates a new IntakeCoral command
     * 
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoIntakeCoral(CoralMaster subsystem) {
      m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopIntake();
    m_subsystem.setStore();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.coralStored();
  }
}
