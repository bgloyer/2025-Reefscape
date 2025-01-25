// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Drive.AlignToTag;
import frc.robot.commands.Drive.PointAtReef;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // define controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_mechController = new CommandXboxController(1);
  
  // The robot's subsystems and commands are defined here...
  private final AlgaeIntake m_intake = new AlgaeIntake();
  private final Claw m_claw = new Claw();
  private final Arm m_arm = new Arm();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_driverController);

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
  
    // drive with controller
    m_robotDrive.setDefaultCommand(Commands.runOnce(() -> m_robotDrive.driveWithController(true), m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings
   */
  private void configureBindings() {
    m_driverController.povUp().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));
    m_driverController.leftTrigger(0.4).whileTrue(new PointAtReef(m_robotDrive));
    m_driverController.x().whileTrue(new AlignToTag(m_robotDrive, AlignToTag.Direction.LEFT));
    m_driverController.b().whileTrue(new AlignToTag(m_robotDrive, AlignToTag.Direction.RIGHT));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
