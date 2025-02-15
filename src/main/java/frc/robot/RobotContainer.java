// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Auto.AutoAlignToTag;
import frc.robot.commands.Auto.AutoIntakeCoral;
import frc.robot.commands.Auto.AutoSetStore;
import frc.robot.commands.CoralIntake.PositionCoral;
import frc.robot.commands.CoralMaster.IntakeCoral;
import frc.robot.commands.CoralMaster.Score;
import frc.robot.commands.CoralMaster.SetLevel;
import frc.robot.commands.CoralMaster.SetStore;
import frc.robot.commands.Drive.AlignToTag;
import frc.robot.commands.Drive.AlignToTag.Direction;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.ClawConstants.WristConstants;
import frc.robot.commands.Drive.PointAtReef;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CoralMaster;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Level;
import frc.robot.util.LimelightHelpers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
  private final Trigger isntDeAlgae = new Trigger(m_coralMaster::scoringButNotDealg);
  private final Trigger readyToBottomDealg = coralStored.negate().or(m_robotDrive::alignedToReef);
  private final Trigger alignedToReef = new Trigger(m_robotDrive::alignedToReef);
  private final Trigger isInTeleop = RobotModeTriggers.teleop();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      // Build an auto chooser. This will use Commands.none() as the default option.
      
      // Configure the trigger bindings
      configureBindings();
      registerAutoCommands();

      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
    
      // drive with controller
      m_robotDrive.setDefaultCommand(Commands.runOnce(() -> m_robotDrive.driveWithController(true), m_robotDrive));
    }
  
    /**
     * Use this method to define your trigger->command mappings
     */
    private void configureBindings() {
      coralStored.negate().and(isntDeAlgae).and(isInTeleop).onTrue(Commands.either(
        new SetStore(m_coralMaster),
        new SetLevel(Level.STORE, m_coralMaster, m_driverController, alignedToReef), 
        new Trigger(() -> m_coralMaster.getLevel() == Level.FOUR)));

      // ------------------ Aidan ----------------------------
      // Intake
      m_driverController.rightTrigger(0.4).whileTrue(new IntakeCoral(m_coralMaster, m_robotDrive));
      m_driverController.rightTrigger(0.4).onFalse(Commands.sequence(
        Commands.runOnce(() -> m_coralMaster.setStore()),
        Commands.waitUntil(m_arm::onTarget),
        new PositionCoral(m_claw).onlyIf(coralStored)));
        
      // Point At Reef
      m_driverController.rightBumper().whileTrue(new PointAtReef(m_robotDrive));
      
      // Score 
      m_driverController.leftBumper().whileTrue(new AlignToTag(m_robotDrive));
        
      // Store everything
      m_driverController.b().onTrue(new SetLevel(Level.STORE, m_coralMaster, m_driverController, alignedToReef));

      // outtake
      m_driverController.a().whileTrue(Commands.startEnd(() -> m_claw.runOuttake(), () -> m_claw.stopIntake()));

      // Reset gyro
      m_driverController.povUp().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));
      m_driverController.povUp().onFalse(Commands.runOnce(() -> LimelightHelpers.SetIMUMode(VisionConstants.LightLightName, 2)));
      m_driverController.back().onTrue(Commands.runOnce(() -> m_robotDrive.roundGyro()));
      
      // ------------------- James ----------------------------
      m_mechController.leftBumper().onTrue(Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.LEFT)));
      m_mechController.rightBumper().onTrue(Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.RIGHT)));

      m_mechController.a().whileTrue(new SetLevel(Level.ONE, m_coralMaster, m_driverController, alignedToReef));
      m_mechController.a().onFalse(new Score(m_coralMaster).until(coralStored.negate()).andThen(new SetStore(m_coralMaster)));

      m_mechController.x().onTrue(new SetLevel(Level.TWO, m_coralMaster, m_driverController, alignedToReef));

      m_mechController.b().and(alignedToReef).onTrue(new SetLevel(Level.THREE, m_coralMaster, m_driverController, alignedToReef));

      m_mechController.y().and(alignedToReef).onTrue(new SetLevel(Level.FOUR, m_coralMaster, m_driverController, alignedToReef));
      
      m_mechController.povUp().whileTrue(new SetLevel(Level.TOPALGAE, m_coralMaster, m_driverController, alignedToReef).alongWith(Commands.runOnce(() -> m_claw.runVoltage(-5))));
      m_mechController.povUp().onFalse(new SetLevel(Level.STORE, m_coralMaster, m_driverController, alignedToReef).alongWith(Commands.runOnce(() -> m_claw.stopIntake())));

      m_mechController.povDown().and(readyToBottomDealg).whileTrue(new SetLevel(Level.BOTTOMALGAE, m_coralMaster, m_driverController, alignedToReef).alongWith(Commands.runOnce(() -> m_claw.runVoltage(-5)))
      );
      m_mechController.povDown().onFalse(new SetLevel(Level.STORE, m_coralMaster, m_driverController, alignedToReef).alongWith(Commands.runOnce(() -> m_claw.stopIntake())));
      
  }
  public void registerAutoCommands() {
    NamedCommands.registerCommand("Auto Intake", new AutoIntakeCoral(m_coralMaster));
    NamedCommands.registerCommand("Set Store", new AutoSetStore(m_coralMaster));
    NamedCommands.registerCommand("Ready Elevator L3", Commands.runOnce(() -> m_coralMaster.setState(ElevatorConstants.BottomDealg, WristConstants.L3)));
    NamedCommands.registerCommand("Ready Elevator L4", Commands.runOnce(() -> m_coralMaster.setState(ElevatorConstants.L4, WristConstants.L4)));

    NamedCommands.registerCommand("Score L2", Commands.parallel(new AlignToTag(m_robotDrive), new SetLevel(Level.TWO, m_coralMaster, m_driverController, alignedToReef)).until(coralStored.negate()));
    NamedCommands.registerCommand("Score L3", Commands.parallel(new AlignToTag(m_robotDrive), new SetLevel(Level.BOTTOMALGAE, m_coralMaster, m_driverController, alignedToReef)).until(coralStored.negate()).andThen(Commands.runOnce(() -> m_claw.runVoltage(-5))));
    NamedCommands.registerCommand("Score L4", Commands.parallel(new AlignToTag(m_robotDrive), new SetLevel(Level.FOUR, m_coralMaster, m_driverController, alignedToReef)).until(coralStored.negate()));
    NamedCommands.registerCommand("Set Left", Commands.runOnce(()-> m_robotDrive.scoringSide = Direction.LEFT));
    NamedCommands.registerCommand("Set Right", Commands.runOnce(()-> m_robotDrive.scoringSide = Direction.RIGHT));
    NamedCommands.registerCommand("PositionCoral", new PositionCoral(m_claw).andThen(() -> m_claw.stopIntake()));
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
    ((PathPlannerAuto)getAutonomousCommand()).getStartingPose().getRotation().getDegrees();
    // LimelightHelpers.SetIMUMode(VisionConstants.LightLightName, 2);
  }

  public void teleopInit() {
    m_arm.resetSetpoint();
    m_arm.setTargetAngle(0);
    m_elevator.resetSetpoint();
    m_claw.resetSetpoint();
    m_claw.stopIntake();
    LimelightHelpers.SetIMUMode(VisionConstants.LightLightName, 2);
  }
}
