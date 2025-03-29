// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive.DriveAutomation;

import static frc.robot.util.Helpers.betterModulus;
import static frc.robot.util.Helpers.tan;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DashboardManager;
import frc.robot.subsystems.CoralMaster.CoralMaster;
import frc.robot.subsystems.Drive.DriveConstants;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.VisionConstants;
import frc.robot.subsystems.Flooral.Flooral;
import frc.robot.subsystems.Flooral.FlooralConstants;

import static frc.robot.util.Helpers.tyToDistance;

import frc.robot.util.Helpers;
import frc.robot.util.LimelightHelpers;

/** An example command that uses an example subsystem. */
public class AutoAlignToStationTag extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_robotDrive;
  private final PIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final String limelightName = VisionConstants.ElevatorLimelightName;
  private final ProfiledPIDController m_turnPID;
  private final CoralMaster m_coralMaster;
  private final Flooral m_flooral;
  private int count = 0;

  /**
   * 
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlignToStationTag(DriveSubsystem subsystem, CoralMaster coralMaster, Flooral flooral) {
    m_robotDrive = subsystem;
    m_flooral = flooral;
    m_xController = new PIDController(DriveConstants.xIntakeTranslationkP, DriveConstants.xIntakeTranslationkI, DriveConstants.xIntakeTranslationkD);
    m_yController = new ProfiledPIDController(DriveConstants.yTranslationkP, DriveConstants.yTranslationkI, DriveConstants.yTranslationkD, new Constraints(4, 2));
    m_turnPID = new ProfiledPIDController(DriveConstants.TurnkP, DriveConstants.TurnkI, DriveConstants.TurnkD, new Constraints(DriveConstants.TurnMaxVelocity, DriveConstants.TurnMaxAccel));
    m_xController.setIZone(0.08); 
    m_yController.setIZone(0.08);
    m_turnPID.enableContinuousInput(0, 360);
    m_coralMaster = coralMaster;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, coralMaster);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    if(DashboardManager.intakeCoralInTheWayOverride) {
      m_coralMaster.setOneCoralAwayIntake();
      m_flooral.setAngle(FlooralConstants.OneCoralAwayStationAngle);
    }
    else {
      m_flooral.setAngle(FlooralConstants.StationAngle);
      m_coralMaster.setIntake();

    }
    m_turnPID.reset(new State(m_robotDrive.getHeading(), -m_robotDrive.getTurnRate()));
    m_yController.reset(new State(tyToDistance(limelightName), m_robotDrive.getSpeeds().vxMetersPerSecond)); // yes y and x are flipped
    m_xController.reset();
    m_yController.setGoal(AligningConstants.IntakeAlignDistance);
    m_yController.setTolerance(0.02);
    m_xController.setSetpoint(m_robotDrive.getStationOffset());
    m_turnPID.setGoal(m_robotDrive.getStationAngle());
    m_robotDrive.setAlignedToReef(false);
    m_robotDrive.setCloseToReef(false);
    // m_robotDrive.setScoringSide(m_robotDrive.getOldScoringSide());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnOutput = m_turnPID.calculate(betterModulus(m_robotDrive.getHeading(), 360));
    if (m_coralMaster.useIntakeAutoAlign() && LimelightHelpers.getTV(limelightName) && tyToDistance(limelightName) < 1.7) {
      double yDistanceFromTag = tyToDistance(limelightName);
      double xInput = Math.abs(yDistanceFromTag) * tan(LimelightHelpers.getTX(limelightName)); // makes align to tag work when not against the wall? 
      double xOutput = 0;
      double yOutput = m_yController.calculate(yDistanceFromTag);
      SmartDashboard.putNumber("Station yOutput", yOutput);
      SmartDashboard.putNumber("robot yvelocity", m_robotDrive.getSpeeds().vxMetersPerSecond);
      if (m_yController.atGoal()) {
        yOutput = 0;
      }
      if (m_turnPID.atGoal()) {
        turnOutput = 0;
      }
      if(Math.abs(betterModulus(m_robotDrive.getHeading(), 360) - m_turnPID.getGoal().position) < 5) {
        xOutput = m_xController.calculate(xInput);
      } 
      m_robotDrive.drive(yOutput + m_yController.getSetpoint().velocity / DriveConstants.kMaxSpeedMetersPerSecond, Math.max(-xOutput, -0.3), turnOutput, false);
      boolean aligned = m_xController.atSetpoint() && m_yController.atGoal();
      m_robotDrive.setAlignedToReef(aligned);
      if(Helpers.isOneCoralAway == false)
        Helpers.isOneCoralAway = coralInTheWay(yOutput);
      SmartDashboard.putBoolean("coral in way", Helpers.isOneCoralAway);
      if(Helpers.isOneCoralAway || DashboardManager.intakeCoralInTheWayOverride) {
        m_coralMaster.setOneCoralAwayIntake();
        m_yController.setGoal(AligningConstants.IntakeOneCoralAwayDistance);
      }
    } else {
      if (DashboardManager.intakeCoralInTheWayOverride) {
        m_coralMaster.setOneCoralAwayIntake();
        m_flooral.setAngle(FlooralConstants.OneCoralAwayStationAngle);
      }
      m_robotDrive.driveWithController(turnOutput, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralMaster.stopIntake();
    m_robotDrive.drive(0,0,0, false);
    Helpers.isOneCoralAway = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_coralMaster.coralStored();
  }

    private boolean coralInTheWay(double yOutput) {
      if(Helpers.isAuto) {
        return false;
      }
      boolean nearOneCoralAway = MathUtil.isNear(AligningConstants.IntakeOneCoralAwayDistance, tyToDistance(limelightName), 0.06);
      boolean notMoving = Math.abs(yOutput) > 0.06 && Math.hypot(m_robotDrive.getSpeeds().vxMetersPerSecond, m_robotDrive.getSpeeds().vyMetersPerSecond) < 0.15; // I pulled these numbers out of my ass
      
      if(nearOneCoralAway && notMoving)
        count++;
      else
        count = 0;
      if(count >= 2)
        return nearOneCoralAway && notMoving;
      return false;
  }
}
