package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants;
import frc.robot.constants.VisionConstants;

public class Helpers {

    public static boolean isBlue;

    public static Pose3d get(int id) {
        return Constants.aprilTags.getTagPose(id).get();
    }

    // public static Alliance getAlliance() {
    //     if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
    //         return Alliance.Red;
    //     return Alliance.Blue;
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
        return VisionConstants.TagToLimelightHeightOffset / tan(LimelightHelpers.getTY(limelightName) + VisionConstants.LimelightMountAngle);
    }
}
