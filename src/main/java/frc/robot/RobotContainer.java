// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CoralMaster.IntakeCoral;
import frc.robot.commands.CoralMaster.Score;
import frc.robot.commands.Drive.AlignToTag;
import frc.robot.commands.Drive.AlignToTag.Direction;
import frc.robot.commands.Drive.PointAtReef;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CoralMaster;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // define controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_mechController = new CommandXboxController(1);
  
  // The robot's subsystems and commands are defined here...
  private final Claw m_claw = new Claw();
  private final Arm m_arm = new Arm();
  private final Elevator m_elevator = new Elevator();
  private final CoralMaster m_coralMaster = new CoralMaster(m_arm, m_elevator, m_claw);
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

      // ------------------ Aidan ----------------------------
      m_driverController.rightTrigger(0.4).whileTrue(new IntakeCoral(m_coralMaster));
      m_driverController.leftTrigger(0.4).whileTrue(new PointAtReef(m_robotDrive));
      
      // Score right
      m_driverController.rightBumper().whileTrue(Commands.sequence(
        new AlignToTag(m_robotDrive, Direction.RIGHT),
        new Score(m_coralMaster, m_driverController)));

      // Score left
      m_driverController.leftBumper().whileTrue(Commands.sequence(
        new AlignToTag(m_robotDrive, Direction.LEFT),
        new Score(m_coralMaster, m_driverController)));
        
      m_driverController.povUp().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));
      
      // ------------------- James ----------------------------
      m_mechController.povDown().onTrue(Commands.runOnce(() -> m_coralMaster.setL1(), m_coralMaster));
      m_mechController.povLeft().onTrue(Commands.runOnce(() -> m_coralMaster.setL2(), m_coralMaster));
      m_mechController.povRight().onTrue(Commands.runOnce(() -> m_coralMaster.setL3(), m_coralMaster));
      m_mechController.povUp().onTrue(Commands.runOnce(() -> m_coralMaster.setL4(), m_coralMaster));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    //An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
