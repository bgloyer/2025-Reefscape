// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CoralIntake.PositionCoral;
import frc.robot.commands.CoralMaster.IntakeCoral;
import frc.robot.commands.CoralMaster.Score;
import frc.robot.commands.CoralMaster.SetLevel;
import frc.robot.commands.Drive.AlignToTag;
import frc.robot.commands.Drive.AlignToTag.Direction;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.ClawConstants.WristConstants;
import frc.robot.constants.Configs.ArmConfig;
import frc.robot.commands.Drive.PointAtReef;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CoralMaster;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Level;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
  private final Trigger coralStored = new Trigger(m_coralMaster::coralStored);
  
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
      m_driverController.rightTrigger(0.4).onFalse(Commands.sequence(
        Commands.runOnce(() -> m_coralMaster.setStore()),
        Commands.waitUntil(m_arm::onTarget),
        new PositionCoral(m_claw).onlyIf(coralStored)));


      m_driverController.rightBumper().whileTrue(new PointAtReef(m_robotDrive));
      
      // Score 
      m_driverController.leftBumper().whileTrue(Commands.sequence(
        new AlignToTag(m_robotDrive),
        Commands.runOnce(() -> m_claw.runVoltage(-4)),
        Commands.waitUntil(coralStored.negate()),
        Commands.runOnce(() -> m_claw.runVoltage(0)),
        new SetLevel(m_coralMaster, Level.STORE)));
        
      m_driverController.povUp().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));

      
      m_driverController.a().whileTrue(Commands.startEnd(() -> m_claw.runVoltage(4), () -> m_claw.runVoltage(0)));
      m_driverController.b().onTrue(new SetLevel(m_coralMaster, Level.STORE));
            
      // ------------------- James ----------------------------
      m_mechController.leftBumper().onTrue(Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.LEFT)));
      m_mechController.rightBumper().onTrue(Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.RIGHT)));

      m_mechController.a().onTrue(new SetLevel(m_coralMaster, Level.ONE));
      m_mechController.x().onTrue(new SetLevel(m_coralMaster, Level.TWO));
      m_mechController.b().onTrue(new SetLevel(m_coralMaster, Level.THREE));
      m_mechController.y().onTrue(new SetLevel(m_coralMaster, Level.FOUR));
  }

  public void registerAutoCommands() {
    NamedCommands.registerCommand("Set L4", Commands.runOnce(() -> m_coralMaster.setL4(), m_coralMaster)); 
    NamedCommands.registerCommand("Set L3", Commands.runOnce(() -> m_coralMaster.setL3(), m_coralMaster)); 
    NamedCommands.registerCommand("Set L2", Commands.runOnce(() -> m_coralMaster.setL2(), m_coralMaster)); 
    NamedCommands.registerCommand("Set L1", Commands.runOnce(() -> m_coralMaster.setL1(), m_coralMaster)); 
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setVortexArmEncoder() {
    m_arm.resetVortexEncoder();
  }

  public void autoInit() {
      m_arm.resetSetpoint();
      m_elevator.resetSetpoint();
      m_claw.resetSetpoint();
  }

  public void teleopInit() {
    m_arm.resetSetpoint();
    m_arm.setTargetAngle(0);
    m_elevator.resetSetpoint();
    m_claw.resetSetpoint();
  }
}
