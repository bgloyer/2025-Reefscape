// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive.DriveAutomation;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class AligningConstants {
    public static final double kNeoFreeSpeedRpm = 5676;
    public static final double kVortexFreeSpeedRpm = 6784;
    public static final double kKrakenFieldOrientedFreeSpeedRpm = 5800;

    // public static final AprilTagFieldLayout aprilTags =  AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static Translation2d blueCenterOfReef = new Translation2d(4.49,4.03); //blue
    public static Translation2d redCenterOfReef = FlippingUtil.flipFieldPosition(AligningConstants.blueCenterOfReef); //blue
    public static final double kNeoVortexkV = 565;
    public static final int FrontCoralLaserCanID = 17;
    public static final int BackCoralLaserCanID = 18;

    public static final double LeftReefOffset = 0.183;//0.181;//0.191
    public static final double RightReefOffset = -0.141;//-0.148;//-0.135
    public static final double MiddleReefOffset = 0.036;

    public static final double ReefAlignTolerance = 0.025;
    public static final double IntakeRightAlignOffset = 0.0675; //0.0716;//0.0907;//0.0894 
    public static final double IntakeLeftAlignOffset = 0.076; //0.104;//0.0708;
    public static final double IntakeAlignDistance = 0.7404;//0.71;
    public static final double IntakeOneCoralAwayDistance = 0.8607;//0.82;
    public static final double ReefOneCoralAwayDistance = 0.6408;//0.635;
    public static final double BlueNetXPosition = 5;
    public static final double RedNetXPosition = 15;
}
