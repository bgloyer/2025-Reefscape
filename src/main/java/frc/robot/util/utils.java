package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants;

public class utils {
    public static Pose3d get(int id) {
        return Constants.aprilTags.getTagPose(id).get();
    }

    public static Alliance getAlliance() {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
            return Alliance.Red;
        return Alliance.Blue;
    }
}
