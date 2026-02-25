package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase{

    Pose3d cameraPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
    public static double cameraDistance;

    public Limelight() {}

    public double get_trig_distance() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);


        double angleToGoalDegrees = Constants.limelightConstants.MountingAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceFromLimelightToGoalInches = (Constants.limelightConstants.goalHeightInches - Constants.limelightConstants.limelightHeightInches) / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }

    public void updateCameraDist() {
        cameraDistance = cameraPose.getZ();
    }

}
