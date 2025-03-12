// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Algae.RunAlgaeIntake;
import frc.robot.commands.Auto.AutoAlignToStationTag;
import frc.robot.commands.Auto.AutoSetStore;
import frc.robot.commands.Auto.OverrideFeedbackIntake;

import static frc.robot.commands.Climber.ClimbFactories.climb;
import static frc.robot.commands.Climber.ClimbFactories.readyClimb;
import static frc.robot.commands.Climber.ClimbFactories.storeClimb;
import frc.robot.commands.CoralIntake.PositionCoral;
import frc.robot.commands.CoralMaster.SetLevel;
import frc.robot.commands.CoralMaster.SetStore;
import frc.robot.commands.Drive.AlignToReef;
import frc.robot.commands.Drive.AlignToReef.Direction;
import frc.robot.constants.AlgaeIntakeConstants;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.BlinkinConstants;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.ClawConstants.WristConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralMaster;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TorSubsystemBase;
import frc.robot.util.Helpers;
import frc.robot.util.Level;
import frc.robot.util.LimelightHelpers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final AlgaeIntake m_algaeIntake = new AlgaeIntake();
  private final Climber m_climber = new Climber();
  private final Blinkin m_blinkin = new Blinkin();
  private final DashboardManager m_dashboardManager = new DashboardManager();

  private final SendableChooser<Command> autoChooser;

  private final Trigger coralStored = new Trigger(m_coralMaster::coralStored);
  private final Trigger algaeStored = new Trigger(m_coralMaster::coralStored);
  private final Trigger isntDeAlgae = new Trigger(m_coralMaster::scoringButNotDealg);
  private final Trigger readyToDealg = coralStored.negate().or(m_robotDrive::closeToReef);
  private final Trigger alignedToReef = new Trigger(m_robotDrive::alignedToReef);
  private final Trigger isInTeleop = RobotModeTriggers.teleop();
  private final Trigger isTopDealgae = new Trigger(m_robotDrive::isTopDealgae);
  private final Trigger atNetHeight = new Trigger(m_elevator::atNetHeight);
  private final Trigger readyToStartScoreSequence = new Trigger(m_robotDrive::closeToReef).or(m_dashboardManager::getUseManualScoring); // idk what to name this

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the trigger bindings
    configureBindings();
    registerAutoCommands();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putBoolean("Mirror Auto?", false);
  
    // drive with controller
    m_robotDrive.setDefaultCommand(Commands.runOnce(() -> m_robotDrive.driveWithController(true), m_robotDrive));
    m_robotDrive.drive(0, 0, 0, true);
    SmartDashboard.putBoolean("switch", false);

    m_dashboardManager.idleModeChooser.addOption("Claw", m_claw);
    m_dashboardManager.idleModeChooser.addOption("Elevator", m_elevator);
    m_dashboardManager.idleModeChooser.addOption("Arm", m_arm);
    m_dashboardManager.idleModeChooser.addOption("Algae Intake", m_algaeIntake);
    }

  
    /**
     * Use this method to define your trigger->command mappings
     */
  private void configureBindings() {
      coralStored.onTrue(m_blinkin.setColor(BlinkinConstants.White));
      coralStored.onFalse(m_blinkin.setColor(BlinkinConstants.Red));
      // coralStored.negate().and(isntDeAlgae).and(isInTeleop).onTrue(Commands.either(
      //   new SetStore(m_coralMaster),
      //   new SetLevel(Level.STORE, m_coralMaster, m_driverController, alignedToReef), 
      //   new Trigger(() -> m_coralMaster.getLevel() == Level.FOUR)));

      // ------------------ Aidan ----------------------------
      // Coral Intake
      m_driverController.rightTrigger(0.4).whileTrue(new AutoAlignToStationTag(m_robotDrive, m_coralMaster));
      m_driverController.rightTrigger(0.4).onFalse(Commands.sequence(
        Commands.runOnce(() -> m_arm.setTargetAngle(ArmConstants.Store), m_arm),
        new WaitCommand(0.1),
        Commands.runOnce(() -> m_claw.setTargetAngle(WristConstants.Store), m_coralMaster)));
        // Commands.waitUntil(m_arm::onTarget),
        // new PositionCoral(m_claw).onlyIf(coralStored)));
              
      // Score
      m_driverController.rightBumper().whileTrue(new AlignToReef(m_robotDrive).alongWith(m_blinkin.setColor(BlinkinConstants.Black)));

      // Algae Intake
      m_driverController.leftTrigger(0.4).whileTrue(new RunAlgaeIntake(AlgaeIntakeConstants.IntakeAngle, AlgaeIntakeConstants.IntakeVoltage, m_algaeIntake));

    Command groundIntake = Commands.sequence(
      Commands.runOnce(() -> m_coralMaster.setState(Level.GROUNDALGAE), m_coralMaster),
      Commands.runOnce(() -> m_claw.runVoltage(7)));

    Command storeAlgae = Commands.sequence(
      Commands.runOnce(() -> m_elevator.setTarget(ElevatorConstants.AlgaeStore), m_elevator),
      Commands.waitUntil(m_elevator::onTarget),
      Commands.runOnce(() -> m_coralMaster.setState(Level.ALGAESTORE), m_coralMaster),
      m_blinkin.setColor(BlinkinConstants.AlgaeHold));

    m_driverController.leftBumper().whileTrue(groundIntake);
    m_driverController.leftBumper().onFalse(Commands.either(
      storeAlgae,
      (Commands.runOnce(() -> m_coralMaster.setStore()).alongWith(Commands.runOnce(() -> m_claw.stopIntake()))), 
      algaeStored));




      // Processor
      // m_driverController.y().whileTrue(new RunAlgaeIntake(AlgaeIntakeConstants.ScoreAngle, AlgaeIntakeConstants.OuttakeVoltage, m_algaeIntake));
      m_driverController.y().whileTrue(new RunAlgaeIntake(AlgaeIntakeConstants.StoreAngle, AlgaeIntakeConstants.OuttakeVoltage, m_algaeIntake));

      // Store everything
      m_driverController.b().onTrue(Commands.runOnce(() -> m_coralMaster.setStore(), m_coralMaster));

      // outtake
      m_driverController.a().whileTrue(Commands.startEnd(() -> m_claw.runIntake(), () -> m_claw.stopIntake()));

      //net score
      Command netScore = Commands.sequence(
        Commands.runOnce(() -> m_coralMaster.setState(Level.NET), m_coralMaster), 
        Commands.waitUntil(atNetHeight),
        Commands.runOnce(() -> m_claw.runVoltage(-8)),
        Commands.runOnce(() -> m_claw.setTargetAngle(WristConstants.AlgaeNetFlick)),
        Commands.waitUntil(m_claw::onTarget),
        Commands.runOnce(() -> m_claw.runVoltage(0)),
        Commands.runOnce(() -> m_coralMaster.setStore(), m_coralMaster),
        Commands.runOnce(() -> System.out.println("getting here"))
      );

      m_driverController.x().onTrue(netScore);
      m_driverController.x().onTrue(m_blinkin.setColor(BlinkinConstants.AlgaeScore));


      // toggle intake mode
      m_driverController.start().onTrue(Commands.runOnce(() -> m_coralMaster.toggleIntakeAutoAlign()));

      // Reset gyro
      m_driverController.povUp().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));
      m_driverController.povUp().onFalse(Commands.runOnce(() -> LimelightHelpers.SetIMUMode(VisionConstants.ReefLightLightName, 2)));
      
      // ------------------- James ----------------------------
      m_mechController.leftBumper().whileTrue(Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.LEFT)));
      m_mechController.rightBumper().whileTrue(Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.RIGHT)));

      m_mechController.a().whileTrue(Commands.runOnce(() -> m_coralMaster.setState(Level.ONE)));
      m_mechController.a().onFalse(Commands.startEnd(() -> m_claw.runVoltage(-6), () -> m_claw.stopIntake()).until(coralStored.negate()).andThen(Commands.waitSeconds(0.4)).andThen(new SetStore(m_coralMaster)));

      m_mechController.x().and(readyToStartScoreSequence).onTrue(Commands.sequence(new SetLevel(Level.TWO, m_coralMaster, m_driverController, alignedToReef).until(coralStored.negate()), new SetLevel(Level.STORE, m_coralMaster, m_driverController, alignedToReef)));

      m_mechController.b().and(readyToStartScoreSequence).onTrue(Commands.sequence(new SetLevel(Level.THREE, m_coralMaster, m_driverController, alignedToReef).until(coralStored.negate()), new SetLevel(Level.STORE, m_coralMaster, m_driverController, alignedToReef)));

      m_mechController.y().and(readyToStartScoreSequence).onTrue(Commands.sequence(new SetLevel(Level.FOUR, m_coralMaster, m_driverController, alignedToReef).until(coralStored.negate()), new SetStore(m_coralMaster)));
      
      Command TopAlgaeGrab = Commands.sequence(
        new SetLevel(Level.FOUR, m_coralMaster, m_driverController, alignedToReef).until(coralStored.negate()),
        Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.MIDDLE)),
        Commands.parallel(
          Commands.runOnce(() -> m_claw.runVoltage(7)),
          new SetLevel(Level.TOPALGAEGRAB, m_coralMaster, m_driverController, alignedToReef)));

        
        Command BottomAlgaeGrab = Commands.sequence(
          new SetLevel(Level.FOUR, m_coralMaster, m_driverController, alignedToReef).until(coralStored.negate()),
          Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.MIDDLE)),
          Commands.parallel(
            Commands.runOnce(() -> m_claw.runVoltage(7)),
            new SetLevel(Level.BOTTOMALGAEGRAB, m_coralMaster, m_driverController, alignedToReef)));

        Command TopAlgaeRoll = Commands.sequence(
          new SetLevel(Level.FOUR, m_coralMaster, m_driverController, alignedToReef).until(coralStored.negate()),
          new SetLevel(Level.TOPALGAEROLL, m_coralMaster, m_driverController, alignedToReef)
          .alongWith(Commands.runOnce(() -> m_claw.runVoltage(-5)).onlyIf
          (coralStored.negate())));

        Command BottomAlgaeRoll = new SetLevel(Level.BOTTOMALGAEROLL, m_coralMaster, m_driverController, alignedToReef).alongWith(Commands.runOnce(() -> m_claw.runVoltage(-5)).onlyIf(coralStored.negate()));
        
        
      
      //Dealgae and store    

      m_mechController.povUp().and(readyToDealg).whileTrue(Commands.either(TopAlgaeGrab, BottomAlgaeGrab, isTopDealgae));
      m_mechController.povUp().onFalse(Commands.either(Commands.runOnce(() -> m_coralMaster.setState(Level.ALGAESTORE), m_coralMaster).alongWith(m_blinkin.setColor(BlinkinConstants.AlgaeHold)), 
        (Commands.runOnce(() -> m_coralMaster.setStore()).alongWith(Commands.runOnce(() -> m_claw.stopIntake()))), 
        algaeStored));

      //Simlultaneous Dealgae and score

      // m_mechController.povDown().and(readyToDealg).whileTrue(new SetLevel(Level.BOTTOMALGAEROLL, m_coralMaster, m_driverController, alignedToReef).alongWith(Commands.runOnce(() -> m_claw.runVoltage(-5)).onlyIf(coralStored.negate())));
      m_mechController.povDown().and(readyToDealg).onTrue(Commands.either(TopAlgaeRoll, BottomAlgaeRoll, isTopDealgae));
      m_mechController.povDown().onFalse(new SetLevel(Level.STORE, m_coralMaster, m_driverController, alignedToReef).alongWith(Commands.runOnce(() -> m_claw.stopIntake())));

      m_mechController.back().onTrue(readyClimb(this));
      m_mechController.start().onTrue(climb(this));
      
      m_mechController.rightStick().onTrue(storeClimb(this));
      m_mechController.rightTrigger(0.3).whileTrue(Commands.run(() -> m_blinkin.setRandom()));

      
      
  }
  public void registerAutoCommands() {
    NamedCommands.registerCommand("Auto Intake", new AutoAlignToStationTag(m_robotDrive, m_coralMaster));
    NamedCommands.registerCommand("Set Store", new AutoSetStore(m_coralMaster));
    NamedCommands.registerCommand("Ready Elevator L3", Commands.runOnce(() -> m_coralMaster.setState(Level.BOTTOMALGAEROLL)));
    NamedCommands.registerCommand("Ready Elevator L4", Commands.runOnce(() -> m_coralMaster.setState(ElevatorConstants.L4, ArmConstants.Store, WristConstants.L4)));
    NamedCommands.registerCommand("Score L2", Commands.parallel(new AlignToReef(m_robotDrive), new SetLevel(Level.TWO, m_coralMaster, m_driverController, alignedToReef)).until(coralStored.negate()));
    NamedCommands.registerCommand("Score L3", Commands.parallel(new AlignToReef(m_robotDrive)).until(alignedToReef.and(m_coralMaster::onTarget)).andThen(Commands.runOnce(() -> m_claw.runVoltage(-5))));
    NamedCommands.registerCommand("Score L4", Commands.parallel(new AlignToReef(m_robotDrive), new SetLevel(Level.FOUR, m_coralMaster, m_driverController, alignedToReef)).until(coralStored.negate()));
    NamedCommands.registerCommand("Set Left", Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.LEFT)));
    NamedCommands.registerCommand("Set Right", Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.RIGHT)));
    // NamedCommands.registerCommand("PositionCoral", new PositionCoral(m_claw).andThen(() -> m_claw.stopIntake()));
    NamedCommands.registerCommand("PositionCoral", Commands.none());
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return new PathPlannerAuto(autoChooser.getSelected().getName(), true);
  }
  
  public void setVortexArmEncoder() {
    m_arm.setZero();
  }

  public void autoInit() {
    Helpers.isAuto = true;
    m_robotDrive.useVision = true;
    m_blinkin.setColor(BlinkinConstants.Red);
    if(m_climber.getAngle() < 30) {
      m_climber.setAngle(ClimbConstants.StoreAngle);
    }
    m_arm.resetSetpoint();
    m_elevator.resetSetpoint();
    m_claw.resetSetpoint();
    ((PathPlannerAuto)getAutonomousCommand()).getStartingPose().getRotation().getDegrees();
    // LimelightHelpers.SetIMUMode(VisionConstants.LightLightName, 2);
  }
  
  public void teleopInit() {
    m_robotDrive.useVision = true;
    Helpers.isAuto = false;
    m_blinkin.setColor(BlinkinConstants.Red);
    m_arm.resetSetpoint();
    m_elevator.resetSetpoint();
    m_claw.resetSetpoint();
    m_claw.stopIntake();
    m_robotDrive.setAlignedToReef(false);
    LimelightHelpers.SetIMUMode(VisionConstants.ReefLightLightName, 2);
  }
  
  public void testPeriodic() {
    if (m_driverController.getHID().getPOV() != -1)
    m_robotDrive.setWheels(m_driverController.getHID().getPOV());

    if(m_driverController.getHID().getAButton())
    m_climber.setAngle(-100);
    else
    m_climber.setVoltage(-12 * MathUtil.applyDeadband(m_driverController.getLeftX(), 0.07));
    
    
    if(m_mechController.getHID().getPOV() == 90) {
      m_claw.runClaw(1);
    } else if (m_mechController.getHID().getPOV() == 270) {
      m_claw.runClaw(-1);
    } else {
      m_claw.runClaw(0);
      }
      
      if (m_mechController.getHID().getYButton()) {
        TorSubsystemBase subsystem = m_dashboardManager.getSelectedSubsystem();
          if (subsystem != null) {
            subsystem.setZero();
          }
      } 
    }
  

  public Claw getClaw() {
    return m_claw;
  }
  
  
  public Arm getArm() {
    return m_arm;
  }


  public Elevator getElevator() {
    return m_elevator;
  }


  public void testInit() {
    m_blinkin.setColorNotCommand(0.67);
    m_arm.stopPid();
    m_algaeIntake.stopPid();
    m_elevator.stopPID();
  }

  public void disabledInit() {
    m_robotDrive.useVision = false;
  }

  public CoralMaster getCoral() {
    return m_coralMaster;
  }

  public Climber getClimber() {
    return m_climber;
  }

  public AlgaeIntake getAlgaeIntake() {
    return m_algaeIntake;
  }


  public void disabledPeriodic() {
    if (m_mechController.getHID().getYButton()) {
      TorSubsystemBase subsystem = m_dashboardManager.getSelectedSubsystem();
        if (subsystem != null) {
          subsystem.setZero();
        }
    } 
    TorSubsystemBase subsystem = m_dashboardManager.getSelectedSubsystem(); 
    if (subsystem != null) {
      SmartDashboard.putBoolean("Is Brake Mode", subsystem.isBrakeMode());
      if(SmartDashboard.getBoolean("Toggle Idle Mode", false)) {
        subsystem.toggleIdleMode();
        SmartDashboard.putBoolean("Toggle Idle Mode", false);
      }
    }
  }
}
