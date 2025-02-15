package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class Vision {

    private SwerveDrivePoseEstimator m_poseEstimator;

    protected Vision(SwerveDrivePoseEstimator poseEstimator) {
        m_poseEstimator = poseEstimator;
    }

    public void updatePoseEstimation(Pigeon2 gyro) {
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation(VisionConstants.LightLightName, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LightLightName);
        if(mt2 == null)
            return;
        if (Math.abs(gyro.getRate()) > 720)  { // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }
        if(LimelightHelpers.getTY(VisionConstants.LightLightName) < -25)
            doRejectUpdate = true;
        if (!doRejectUpdate) {
            m_poseEstimator.setVisionMeasurementStdDevs(VisionConstants.VisionStdDev);
            m_poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
        }

    }
}
