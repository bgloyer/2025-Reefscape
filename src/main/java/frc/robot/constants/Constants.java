// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kNeoFreeSpeedRpm = 5676;
    public static final double kVortexFreeSpeedRpm = 6784;
    public static final double kKrakenFieldOrientedFreeSpeedRpm = 5800;

    // public static final AprilTagFieldLayout aprilTags =  AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static Translation2d blueCenterOfReef = new Translation2d(4.49,4.03); //blue
    public static final double kNeoVortexkV = 565;
    public static final int BackCoralLaserCanID = 17;
    public static final int FrontCoralLaserCanID = 18;

    public static final double LeftReefOffset = 0.181;//0.191
    public static final double RightReefOffset = -0.148;//-0.135
    public static final double ReefAlignTolerance = 0.025;
    public static final double IntakeAlignOffset = -0.0894;
    public static final double IntakeAlignDistance = 0.68;
}
