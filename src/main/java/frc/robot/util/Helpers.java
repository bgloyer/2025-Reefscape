package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;

public class Helpers {

    public static boolean isBlue = false;

    // public static Pose3d get(int id) {
    //     return Constants.aprilTags.getTagPose(id).get();
    // }

    public static double betterModulus(double x, double y) {
        return (x % y + y) % y;
    }

    /**
    *   @param degrees input angle in degrees
    *   @return tan of the angle
    */
    public static double tan(double degrees) {
        return Math.tan(Math.toRadians(degrees));
    }

    /**
     * ONLY WORKS FOR REEF TAGS
     * @return distance from limelight to tag
     */
    public static double tyToDistance(String limelightName) {
        switch (limelightName) {
            case VisionConstants.ReefLightLightName:
                return VisionConstants.TagToLimelightHeightOffset / tan(LimelightHelpers.getTY(limelightName) + VisionConstants.LimelightMountAngle);
            case VisionConstants.ElevatorLimelightName:
                return 0.4649 / tan(LimelightHelpers.getTY(limelightName) + 28);
            default:
                return 0;
        }
    }
}
