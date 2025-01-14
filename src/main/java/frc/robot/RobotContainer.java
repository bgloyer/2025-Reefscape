// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Drive.PointAtReef;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.DriveSubsystem;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // define controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_mechController = new CommandXboxController(1);
  
  // The robot's subsystems and commands are defined here...
  private final AlgaeIntake m_intake = new AlgaeIntake();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_driverController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  
    // drive with controller
    m_robotDrive.setDefaultCommand(Commands.runOnce(() -> m_robotDrive.driveWithController(true),m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings
   */
  private void configureBindings() {
    m_driverController.povUp().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));

    m_driverController.leftTrigger(0.4).whileTrue(new PointAtReef(m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("Auto");
  }
}
