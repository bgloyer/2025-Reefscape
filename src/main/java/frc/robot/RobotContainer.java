// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Auto.AutoSetStore;
import frc.robot.subsystems.TorSubsystemBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Blinkin.Blinkin;
import frc.robot.subsystems.Blinkin.BlinkinConstants;
import frc.robot.subsystems.Claw.Claw;
import frc.robot.subsystems.Claw.ClawConstants.CoralIntakeConstants;
import frc.robot.subsystems.Claw.ClawConstants.WristConstants;
import frc.robot.subsystems.Climber.ClimbConstants;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.CoralMaster.CoralMaster;
import frc.robot.subsystems.CoralMaster.SetLevel;
import frc.robot.subsystems.CoralMaster.SetStore;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.VisionConstants;
import frc.robot.subsystems.Drive.DriveAutomation.AlignToNet;
import frc.robot.subsystems.Drive.DriveAutomation.AlignToReef;
import frc.robot.subsystems.Drive.DriveAutomation.AutoAlignToStationTag;
import frc.robot.subsystems.Drive.DriveAutomation.AlignToReef.Direction;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Flooral.Flooral;
import frc.robot.subsystems.Flooral.FlooralConstants;
import frc.robot.util.Helpers;
import frc.robot.util.Level;
import frc.robot.util.LimelightHelpers;

import static frc.robot.subsystems.Climber.ClimbFactories.climb;
import static frc.robot.subsystems.Climber.ClimbFactories.readyClimb;
import static frc.robot.subsystems.Climber.ClimbFactories.storeClimb;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final Flooral m_flooral = new Flooral();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_driverController);
  private final Climber m_climber = new Climber();
  private final Blinkin m_blinkin = new Blinkin();
  private final DashboardManager m_dashboardManager = new DashboardManager();

  private final SendableChooser<Command> autoChooser;

  public final Trigger coralStored = new Trigger(m_coralMaster::coralStored);
  public final Trigger flooralStored = new Trigger(m_flooral::coralStored);
  public final Trigger flooralPivotAtStore = new Trigger(m_flooral::atStore);
  private final Trigger algaeStored = new Trigger(m_coralMaster::coralStored);
  private final Trigger readyToDealg = coralStored.negate().or(m_robotDrive::closeToReef);
  public final Trigger alignedToReef = new Trigger(m_robotDrive::alignedToReef);
  private final Trigger alignedToMiddle = new Trigger(m_coralMaster::alignedToMiddle);
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
  
    // drive with controller
    m_robotDrive.setDefaultCommand(Commands.runOnce(() -> m_robotDrive.driveWithController(true), m_robotDrive));
    m_robotDrive.drive(0, 0, 0, true);
    SmartDashboard.putBoolean("switch", false);

    m_dashboardManager.idleModeChooser.addOption("Claw", m_claw);
    m_dashboardManager.idleModeChooser.addOption("Elevator", m_elevator);
    m_dashboardManager.idleModeChooser.addOption("Arm", m_arm);
    }

  
    /**
     * Use this method to define your trigger->command mappings
     */
  private void configureBindings() {
    coralStored.onTrue(m_blinkin.setColor(BlinkinConstants.White));
    coralStored.onFalse(m_blinkin.setColor(BlinkinConstants.Red));
    flooralStored.and(m_flooral::getHoldingCoralState).and(coralStored.negate()).and(() -> m_elevator.getHeight() < 0.1).onTrue(handOff());


    // ------------------ Aidan ----------------------------

    
    // Coral Intake
    m_driverController.rightTrigger(0.4).whileTrue(new AutoAlignToStationTag(m_robotDrive, m_coralMaster, m_flooral));
    m_driverController.rightTrigger(0.4).onFalse(Commands.sequence(
      Commands.runOnce(() -> m_arm.setTargetAngle(ArmConstants.Store), m_arm),
      new WaitCommand(0.1),
      Commands.runOnce(() -> m_claw.setTargetAngle(WristConstants.Store), m_coralMaster),
      Commands.runOnce(() -> m_flooral.setAngle(FlooralConstants.HandoffAngle), m_flooral)));
      
    // Score
    m_driverController.rightBumper().whileTrue(new AlignToReef(m_robotDrive).alongWith(m_blinkin.setColor(BlinkinConstants.Black)));
      
    // Store everything
    m_driverController.b().onTrue(Commands.runOnce(() -> m_coralMaster.setStore(), m_coralMaster).alongWith(m_flooral.setStore()));

    // outtake
    m_driverController.a().whileTrue(Commands.startEnd(() -> m_claw.runIntake(), () -> m_claw.stopIntake()));
    m_driverController.y().whileTrue(Commands.startEnd(() -> m_claw.runVoltage(-CoralIntakeConstants.IntakeVoltage), () -> m_claw.stopIntake()));
    
    //net score
    Command netScore = Commands.sequence(
      new AlignToNet(m_robotDrive),
      Commands.runOnce(() -> m_coralMaster.setState(Level.NET), m_coralMaster), 
      Commands.waitUntil(atNetHeight),
      Commands.runOnce(() -> m_claw.runVoltage(-8)),
      Commands.runOnce(() -> m_claw.setTargetAngle(WristConstants.AlgaeNetFlick)),
      Commands.waitUntil(m_claw::onTarget),
      Commands.runOnce(() -> m_claw.runVoltage(0)),
      Commands.runOnce(() -> m_coralMaster.setStore(), m_coralMaster)
      );
      
    m_driverController.x().whileTrue(netScore);      
    m_driverController.x().onTrue(m_blinkin.setColor(BlinkinConstants.AlgaeScore));
    
    // Intake Coral from Floor
    m_driverController.leftTrigger(0.4).onTrue(flooralIntakeSequence());
    m_driverController.leftTrigger(0.4).onFalse(Commands.either(
      Commands.either(
        Commands.runOnce(() -> m_flooral.setHoldingCoralState(true)), 
        handOff(),  coralStored),
      m_flooral.setStore().alongWith(Commands.runOnce(() -> m_coralMaster.setState(Level.STORE)).onlyIf(coralStored.negate())),
      flooralStored));

    // toggle intake mode
    m_driverController.start().onTrue(Commands.runOnce(() -> m_coralMaster.toggleIntakeAutoAlign()));

    Command groundIntake = Commands.sequence(
      Commands.runOnce(() -> m_coralMaster.setState(Level.GROUNDALGAE), m_coralMaster),
      Commands.runOnce(() -> m_claw.runVoltage(7)));

    Command storeAlgae = Commands.sequence(
      Commands.runOnce(() -> m_elevator.setTarget(0.25), m_elevator),
      // Commands.waitUntil(m_elevator::onTarget),
      // Commands.runOnce(() -> m_claw.runVoltage(-8)),
      // Commands.waitSeconds(0.3),
      // Commands.runOnce(() -> m_claw.runVoltage(-8)),
      // Commands.runOnce(() -> m_coralMaster.setState(Level.ALGAESTORE), m_coralMaster),
      m_blinkin.setColor(BlinkinConstants.AlgaeHold));

    m_driverController.leftBumper().whileTrue(groundIntake);
    m_driverController.leftBumper().onFalse(Commands.either(
      storeAlgae,
      (Commands.runOnce(() -> m_coralMaster.setStore()).alongWith(Commands.runOnce(() -> m_claw.stopIntake()))), 
      algaeStored));

    // Reset gyro
    m_driverController.povUp().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));
    m_driverController.povUp().onFalse(Commands.runOnce(() -> LimelightHelpers.SetIMUMode(VisionConstants.ReefLightLightName, 2)));

    
    // ------------------- James ----------------------------
    m_mechController.leftBumper().whileTrue(Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.LEFT)));
    m_mechController.rightBumper().whileTrue(Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.RIGHT)));

    m_mechController.a().whileTrue(Commands.runOnce(() -> m_coralMaster.setState(Level.ONE)));
    m_mechController.a().onFalse(Commands.startEnd(() -> m_claw.runVoltage(6), () -> m_claw.stopIntake()).until(coralStored.negate()).andThen(Commands.waitSeconds(0.4)).andThen(new SetStore(m_coralMaster)));

    m_mechController.x().and(readyToStartScoreSequence).onTrue(Commands.sequence(new SetLevel(Level.TWO, m_coralMaster, alignedToReef).until(coralStored.negate()), new SetLevel(Level.STORE, m_coralMaster, alignedToReef)));

    m_mechController.b().and(readyToStartScoreSequence).onTrue(Commands.sequence(new SetLevel(Level.THREE, m_coralMaster, alignedToReef).until(coralStored.negate()), new SetLevel(Level.STORE, m_coralMaster, alignedToReef)));

    m_mechController.y().and(readyToStartScoreSequence).onTrue(Commands.sequence(new SetLevel(Level.FOUR, m_coralMaster, alignedToReef).until(coralStored.negate()), new SetStore(m_coralMaster)));

    Command TopAlgaeGrab = Commands.sequence(
      Commands.runOnce(() -> m_coralMaster.setranL4(m_coralMaster.coralStored())),
      new SetLevel(Level.FOUR, m_coralMaster, alignedToReef).until(coralStored.negate()),
      Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.MIDDLE)),
      Commands.waitUntil(alignedToMiddle.or(m_coralMaster::getRanL4)),
      Commands.parallel(
        Commands.runOnce(() -> m_claw.runVoltage(7)),
        new SetLevel(Level.TOPALGAEGRAB, m_coralMaster, alignedToReef)));

      
    Command BottomAlgaeGrab = Commands.sequence(
        Commands.runOnce(() -> m_coralMaster.setranL4(m_coralMaster.coralStored())),
        new SetLevel(Level.FOUR, m_coralMaster, alignedToReef).until(coralStored.negate()),
        Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.MIDDLE)),
        Commands.waitUntil(alignedToMiddle.or(m_coralMaster::getRanL4)),
        Commands.parallel(
          Commands.runOnce(() -> m_claw.runVoltage(7)),
          new SetLevel(Level.BOTTOMALGAEGRAB, m_coralMaster, alignedToReef)));

    Command TopAlgaeRoll = Commands.sequence(
        new SetLevel(Level.FOUR, m_coralMaster, alignedToReef).until(coralStored.negate()),
        new SetLevel(Level.TOPALGAEROLL, m_coralMaster, alignedToReef)
        .alongWith(Commands.runOnce(() -> m_claw.runVoltage(-5)).onlyIf
        (coralStored.negate())));

    Command BottomAlgaeRoll = new SetLevel(Level.BOTTOMALGAEROLL, m_coralMaster, alignedToReef).alongWith(Commands.runOnce(() -> m_claw.runVoltage(-5)).onlyIf(coralStored.negate()));
      
      
    
    //Dealgae and store   
    m_mechController.povUp().and(readyToDealg).whileTrue(Commands.either(TopAlgaeGrab, BottomAlgaeGrab, isTopDealgae));
    m_mechController.povUp().onFalse(Commands.either(Commands.runOnce(() -> m_coralMaster.setState(Level.ALGAESTORE), m_coralMaster).alongWith(m_blinkin.setColor(BlinkinConstants.AlgaeHold)), 
      (Commands.runOnce(() -> m_coralMaster.setStore()).alongWith(Commands.runOnce(() -> m_claw.stopIntake()))), 
      algaeStored));

    //Simlultaneous Dealgae and score

    // m_mechController.povDown().and(readyToDealg).whileTrue(new SetLevel(Level.BOTTOMALGAEROLL, m_coralMaster, alignedToReef).alongWith(Commands.runOnce(() -> m_claw.runVoltage(-5)).onlyIf(coralStored.negate())));
    m_mechController.povDown().and(readyToDealg).onTrue(Commands.either(TopAlgaeRoll, BottomAlgaeRoll, isTopDealgae));
    m_mechController.povDown().onFalse(new SetLevel(Level.STORE, m_coralMaster, alignedToReef).alongWith(Commands.runOnce(() -> m_claw.stopIntake())));

    m_mechController.back().onTrue(readyClimb(this));
    m_mechController.start().onTrue(climb(this));
    
    m_mechController.rightStick().onTrue(storeClimb(this));
    m_mechController.rightTrigger(0.3).whileTrue(Commands.run(() -> m_blinkin.setRandom()));

      
      
  }
  public void registerAutoCommands() {
    NamedCommands.registerCommand("Auto Intake", new AutoAlignToStationTag(m_robotDrive, m_coralMaster, m_flooral));
    NamedCommands.registerCommand("Set Store", new AutoSetStore(m_coralMaster));
    NamedCommands.registerCommand("Ready Elevator L3", Commands.runOnce(() -> m_coralMaster.setState(Level.BOTTOMALGAEROLL)));
    NamedCommands.registerCommand("Ready Elevator L4", Commands.runOnce(() -> m_coralMaster.setState(ElevatorConstants.L4, ArmConstants.Store, WristConstants.L4)));
    NamedCommands.registerCommand("Score L2", Commands.parallel(new AlignToReef(m_robotDrive), new SetLevel(Level.TWO, m_coralMaster, alignedToReef)).until(coralStored.negate()));
    NamedCommands.registerCommand("Score L3", Commands.parallel(new AlignToReef(m_robotDrive)).until(alignedToReef.and(m_coralMaster::onTarget)).andThen(Commands.runOnce(() -> m_claw.runVoltage(-5))));
    NamedCommands.registerCommand("Score L4", Commands.parallel(new AlignToReef(m_robotDrive), new SetLevel(Level.FOUR, m_coralMaster, alignedToReef)).until(coralStored.negate()));
    NamedCommands.registerCommand("Set Left", Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.LEFT)));
    NamedCommands.registerCommand("Set Right", Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.RIGHT)));
    // NamedCommands.registerCommand("PositionCoral", new PositionCoral(m_claw).andThen(() -> m_claw.stopIntake()));
    NamedCommands.registerCommand("PositionCoral", Commands.none());
  }
  
  public Command flooralIntakeSequence() {
    return Commands.sequence(
      Commands.runOnce(() -> m_flooral.setIntake(), m_flooral),
      Commands.runOnce(() -> m_coralMaster.setState(Level.FLOORALHANDOFF), m_coralMaster).onlyIf(coralStored.negate()),
      Commands.waitUntil(m_flooral::coralStored),
      Commands.waitSeconds(0.06),
      Commands.runOnce(() -> m_flooral.holdCoral()),
      Commands.runOnce(() -> m_flooral.setAngle(FlooralConstants.CoralStore)));
  }

  public Command handOff() {
    return Commands.sequence(
      Commands.runOnce(() -> m_flooral.setAngle(FlooralConstants.CoralStore)).onlyIf(flooralStored),
      Commands.runOnce(() -> m_flooral.holdCoral()),
      Commands.runOnce(() -> m_coralMaster.setState(Level.FLOORALHANDOFF), m_coralMaster),
      Commands.waitUntil(m_coralMaster::onTarget),
      Commands.runOnce(() -> m_flooral.setAngle(FlooralConstants.HandoffAngle), m_flooral),
      Commands.waitUntil(m_flooral::onTarget),
      Commands.runOnce(() -> m_claw.runVoltage(CoralIntakeConstants.HandOffVoltage)),
      Commands.runOnce(() -> m_flooral.setVoltage(FlooralConstants.HandoffVoltage, FlooralConstants.HandoffVoltage), m_flooral),
      Commands.waitUntil(coralStored),
      Commands.runOnce(() -> m_claw.stopIntake()),
      m_flooral.stopMotor(),
      Commands.runOnce(() -> m_flooral.setAngle(FlooralConstants.StationAngle)),
      Commands.runOnce(() -> m_coralMaster.setStore(), m_coralMaster),
      Commands.runOnce(() -> m_flooral.setHoldingCoralState(false))
    );
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    return new WaitCommand(0.01).andThen(autoChooser.getSelected());
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
    m_flooral.setStore();
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

  public Flooral getFlooral() {
    return m_flooral;
  }


  public void testInit() {
    m_blinkin.setColorNotCommand(0.67);
    m_arm.stopPid();
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
  
  public DriveSubsystem getDrive() {
    return m_robotDrive;
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
