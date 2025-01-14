package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.Constants;

public class AprilTag {
    public static Pose3d get(int id) {
        return Constants.aprilTags.getTagPose(id).get();
    }
}
