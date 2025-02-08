// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CoralMaster.IntakeCoral;
import frc.robot.commands.CoralMaster.Score;
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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
  private final Claw m_claw = new Claw();
  private final Arm m_arm = new Arm();
  private final Elevator m_elevator = new Elevator();
  // private final CoralMaster m_coralMaster = new CoralMaster(m_arm, m_elevator, m_claw);
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
      // m_driverController.rightTrigger(0.4).whileTrue(new IntakeCoral(m_coralMaster));
      m_driverController.leftTrigger(0.4).whileTrue(new PointAtReef(m_robotDrive));
      
      // Score right
      m_driverController.rightBumper().whileTrue(Commands.sequence(
        new AlignToTag(m_robotDrive, Direction.RIGHT),
        Commands.runOnce(() -> m_claw.runVoltage(-5.5))));
        // new Score(m_coralMaster, m_driverController)));

      // Score left
      m_driverController.leftBumper().whileTrue(Commands.sequence(
        new AlignToTag(m_robotDrive, Direction.LEFT),
        Commands.runOnce(() -> m_claw.runVoltage(-5.5))));
        // new Score(m_coralMaster, m_driverController)));
        
      m_driverController.povUp().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));

      
      m_driverController.a().whileTrue(Commands.startEnd(() -> m_claw.runVoltage(4), () -> m_claw.runVoltage(0)));
      m_driverController.b().whileTrue(Commands.startEnd(() -> m_claw.runVoltage(-5.5), () -> m_claw.runVoltage(0)));

      
      m_driverController.x().onTrue(Commands.parallel(
        Commands.runOnce(() -> m_arm.setTargetAngle(ArmConstants.L2)),
        Commands.runOnce(() -> m_claw.setTargetAngle(WristConstants.L2))));
      
      
      m_driverController.y().onTrue(Commands.runOnce(() -> m_arm.setTargetAngle(0)));

      m_driverController.rightTrigger(0.4).onTrue(Commands.parallel(
        Commands.runOnce(() -> m_arm.setTargetAngle(37.6)),
        Commands.runOnce(() -> m_claw.setTargetAngle(21.27))));

      

      
      m_mechController.x().onTrue(Commands.runOnce(() -> m_claw.setTargetAngle(0)));
      m_mechController.y().onTrue(Commands.runOnce(() -> m_claw.setTargetAngle(90)));
      m_mechController.b().onTrue(Commands.runOnce(() -> m_claw.setTargetAngle(120)));

      // grab a kG value from smartdashboard and apply the voltage to the elevator motors
      
      // ------------------- James ----------------------------
      // m_mechController.povDown().onTrue(Commands.runOnce(() -> m_coralMaster.setL1(), m_coralMaster));
      // m_mechController.povLeft().onTrue(Commands.runOnce(() -> m_coralMaster.setL2(), m_coralMaster));
      // m_mechController.povRight().onTrue(Commands.runOnce(() -> m_coralMaster.setL3(), m_coralMaster));
      // m_mechController.povUp().onTrue(Commands.runOnce(() -> m_coralMaster.setL4(), m_coralMaster));
  }

  public void registerAutoCommands() {
    // NamedCommands.registerCommand("Set L4", Commands.runOnce(() -> m_coralMaster.setL4(), m_coralMaster)); 
    // NamedCommands.registerCommand("Set L3", Commands.runOnce(() -> m_coralMaster.setL3(), m_coralMaster)); 
    // NamedCommands.registerCommand("Set L2", Commands.runOnce(() -> m_coralMaster.setL2(), m_coralMaster)); 
    // NamedCommands.registerCommand("Set L1", Commands.runOnce(() -> m_coralMaster.setL1(), m_coralMaster)); 
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
