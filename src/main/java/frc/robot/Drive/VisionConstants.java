package frc.robot.Drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    
    public static final String ReefLightLightName = "limelight-four";
    public static final double TagToLimelightHeightOffset = 0.135892; // meters; check if this is right
    public static final double LimelightMountAngle = 28.5; // check this

    /* Standard deviations of the pose estimate (x position 
    * in meters, y position in meters, and heading in radians). 
    * Increase these numbers to trust your state estimate less.
    **/
    public static Matrix<N3, N1> StateStdDev = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));


    /*
    *  Standard deviations of the vision pose measurement (x position
    *  in meters, y position in meters, and heading in radians). Increase 
    *  thes numbers to trustthe vision pose measurement less.
    **/
    public static Matrix<N3, N1> VisionStdDev = VecBuilder.fill(.7, .7, 9999999);

    public static final String ElevatorLimelightName = "limelight-elev";
}
