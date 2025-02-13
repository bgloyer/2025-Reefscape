// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Drive.AlignToTag.Direction;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.Helpers;
import frc.robot.util.LimelightHelpers;

import static frc.robot.util.Helpers.betterModulus;
import static frc.robot.util.Helpers.isBlue;
import static frc.robot.util.Helpers.tan;
import static frc.robot.util.Helpers.tyToDistance;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveSubsystem extends SubsystemBase {
  // Create KrakenSwerveModules
  private final KrakenSwerveModule m_frontLeft = new KrakenSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final KrakenSwerveModule m_frontRight = new KrakenSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final KrakenSwerveModule m_rearLeft = new KrakenSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final KrakenSwerveModule m_rearRight = new KrakenSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private KrakenSwerveModule[] modules = new KrakenSwerveModule[] {
      m_frontLeft,
      m_frontRight,
      m_rearLeft,
      m_rearRight
  };

  // The gyro sensor
  public final Pigeon2 m_gyro = new Pigeon2(16, "rio");
  private Field2d m_field2d = new Field2d();
  private SwerveDriveKinematics kinematics = DriveConstants.kDriveKinematics;
  private Vision m_vision;
  public CommandXboxController m_driverController;

  private boolean useVision = true;
  // Odometry class for tracking robot pose
  private SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d(0,0, Rotation2d.fromDegrees(0)),
      VisionConstants.StateStdDev,
      VisionConstants.VisionStdDev);

public Direction scoringSide = Direction.RIGHT;

private boolean aligned = false;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(CommandXboxController controller) {
    m_driverController = controller;
    m_gyro.setYaw(0);
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                                // ChassisSpeeds. Also optionally outputs individual
                                                                // module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic drive trains
              new PIDConstants(24, 0.0, 0.0), // Translation PID constants
              new PIDConstants(10.0, 0.0, 0.0) // Rotation PID constants
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            // var alliance = DriverStation.getAlliance();
            // if (alliance.isPresent()) {
            //   return alliance.get() == DriverStation.Alliance.Red;
            // }
            // return false;
            return true;
          },
          this);
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    m_vision = new Vision(m_odometry);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance m", Helpers.tyToDistance(VisionConstants.LightLightName));
    SmartDashboard.putNumber("X Offset Distance m ", Helpers.tyToDistance(VisionConstants.LightLightName) * Helpers.tan(LimelightHelpers.getTX(VisionConstants.LightLightName)));
    getAngleToReef();
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition() });

    m_field2d.setRobotPose(m_odometry.getEstimatedPosition());

    if (useVision) {
      m_vision.updatePoseEstimation(m_gyro);
    }

    smartDashboardPrints();
  }

  private void smartDashboardPrints() {
    SmartDashboard.putBoolean("Aligned to reef", alignedToReef());
    SmartDashboard.putData("Robot Field", m_field2d);
    SmartDashboard.putNumber("Odometry X", m_odometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Odometry Y", m_odometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Robot Yaw", getHeading());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void driveWithController(boolean fieldRelative) {
    drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), DriveConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), DriveConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), DriveConstants.kDriveDeadband),
        true);
  }

  /**
   * Method to drive the robot using joystick info and given rotation output.
   *
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void driveWithController(double rot, boolean fieldRelative) {
    drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), DriveConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), DriveConstants.kDriveDeadband),
        rot,
        true);
  }
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setWheels(double angle) {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(angle)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public boolean wheelsOnTarget() {
    return Math.abs(betterModulus(m_frontLeft.getPosition().angle.getDegrees(), 180) - 90) < 10;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_odometry.resetRotation(Rotation2d.fromDegrees(Helpers.isBlue ? 0 : 180));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, AutoConstants.kMaxSpeedMetersPerSecond);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(targetStates[i]);
    }
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

    // Since AutoBuilder is configured, we can use it to build pathfinding commands

    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0));
  }

  public double getAngleToReef() {
    Translation2d centerOfReef = Constants.blueCenterOfReef;
    if (!isBlue) {
      centerOfReef = FlippingUtil.flipFieldPosition(Constants.blueCenterOfReef);
    }
    double angle = centerOfReef.minus(getPose().getTranslation()).getAngle().getDegrees();
    
    return 180 + Math.round(angle / 60.0) * 60;
  }

  public void setScoringSide(Direction dir) {
    scoringSide = dir;
  }

  public void setAlignedToReef(boolean val) {
    aligned = val;
  }

  public boolean alignedToReef() {
    return aligned;
  }

  public double getStationAngle() {
    if (m_odometry.getEstimatedPosition().getY() < 4) {
      return -54.011;
    } else {
      return 54.011;
    }
  }
}