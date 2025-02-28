// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import static frc.robot.util.Helpers.betterModulus;
import static frc.robot.util.Helpers.tan;
import static frc.robot.util.Helpers.tyToDistance;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Helpers;
import frc.robot.util.LimelightHelpers;

/** An example command that uses an example subsystem. */
public class AlignToReef extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_robotDrive;
  private final ProfiledPIDController m_xController;
  private final ProfiledPIDController m_yController;
  private final String limelightName = VisionConstants.ReefLightLightName;
  private final ProfiledPIDController m_turnPID;
  public enum Direction {
    LEFT, RIGHT
  }

  /** 
   * Aligns robot to the reef for scoring
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToReef(DriveSubsystem subsystem) {
    m_robotDrive = subsystem;
    m_xController = new ProfiledPIDController(DriveConstants.xTranslationkP, DriveConstants.xTranslationkI, DriveConstants.xTranslationkD, new Constraints(2, 2.5));
    m_yController = new ProfiledPIDController(DriveConstants.yTranslationkP, DriveConstants.yTranslationkI, DriveConstants.yTranslationkD, new Constraints(4, 2.5));

    m_turnPID = new ProfiledPIDController(DriveConstants.TurnkP, DriveConstants.TurnkI, DriveConstants.TurnkD, new Constraints(DriveConstants.TurnMaxVelocity, DriveConstants.TurnMaxAccel));
    m_xController.setIZone(0.08); // 0.04
    m_yController.setIZone(0.08);
    m_turnPID.enableContinuousInput(0, 360);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnPID.reset(new State(m_robotDrive.getHeading(), -m_robotDrive.getTurnRate()));
    m_turnPID.setTolerance(1.5);
    m_turnPID.setGoal(m_robotDrive.getAngleToReef());
    m_yController.setTolerance(0.01);
    m_xController.setTolerance(Constants.ReefAlignTolerance);
    m_yController.reset(new State(tyToDistance(limelightName), -m_robotDrive.getSpeeds().vxMetersPerSecond)); // yes y and x are flipped
    m_xController.reset(new State(tyToDistance(limelightName) * tan(LimelightHelpers.getTX(limelightName)), m_robotDrive.getSpeeds().vyMetersPerSecond));
    m_yController.setGoal(0.5); // one coral away: 0.62
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    switch (m_robotDrive.scoringSide) {
      case LEFT:
        m_xController.setGoal(Constants.LeftReefOffset); // mid 0.2165 
        break;
        case RIGHT:
        m_xController.setGoal(Constants.RightReefOffset);
        break;
      }
    double turnOutput = m_turnPID.calculate(betterModulus(m_robotDrive.getHeading(), 360));
    if (LimelightHelpers.getTV(limelightName)) {
      double yDistanceFromTag = tyToDistance(limelightName);
      double xInput = yDistanceFromTag * tan(LimelightHelpers.getTX(limelightName)); // makes align to tag work when not against the wall? 
      double xOutput = 0;
      double yOutput = -m_yController.calculate(yDistanceFromTag);
      
      if (m_yController.atGoal()) {
        yOutput = 0;
      }
      if (m_turnPID.atGoal()) {
        turnOutput = 0;
      }
      if(Math.abs(betterModulus(m_robotDrive.getHeading(), 360) - m_turnPID.getGoal().position) < 5) {
        xOutput = m_xController.calculate(xInput);
      } else {
        m_xController.reset(new State(tyToDistance(limelightName) * tan(LimelightHelpers.getTX(limelightName)), m_robotDrive.getSpeeds().vyMetersPerSecond));
      }
      m_robotDrive.drive(Math.min(yOutput, 0.3), Math.min(xOutput, 0.3), turnOutput, false);
      boolean aligned = m_xController.atGoal() && m_yController.atGoal();
      m_robotDrive.setAlignedToReef(aligned);
      m_robotDrive.setCloseToReef(Math.abs(yDistanceFromTag - m_yController.getGoal().position) < 0.7);
      if(Helpers.isOneCoralAway == false)
        Helpers.isOneCoralAway = coralInTheWay(yOutput);
      SmartDashboard.putBoolean("coral in way", Helpers.isOneCoralAway);
      if(Helpers.isOneCoralAway) {
        m_yController.setGoal(Constants.ReefOneCoralAwayDistance);
      }
    } else
      m_robotDrive.drive(0,0,turnOutput, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.drive(0,0,0, false);
    Helpers.isOneCoralAway = false;
    m_robotDrive.setCloseToReef(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean coralInTheWay(double yOutput) {
    boolean nearOneCoralAway = MathUtil.isNear(Constants.ReefOneCoralAwayDistance, tyToDistance(limelightName), 0.06);
    boolean notMoving = Math.abs(yOutput) > 0.05 && Math.abs(m_robotDrive.getSpeeds().vxMetersPerSecond) < 0.1; // I pulled these numbers out of my ass

    return nearOneCoralAway && notMoving;
  }

}
