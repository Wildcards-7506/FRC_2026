package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

    //PID for distance
    public final PIDController xController = new PIDController(1.5, 0.0, 0.01);
    public final PIDController yController = new PIDController(1.5, 0.0, 0.01);
    public final PIDController thetaController = new PIDController(1.5, 0.0, 0.01);

    public Limelight() {
        //Distance code: Untested
        xController.setTolerance(0.02);
        yController.setTolerance(0.02);


        thetaController.enableContinuousInput(-180, 180);
        thetaController.setTolerance(2.0); // 2 degree tolerance
    }

    public double get_trig_distance() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);


        double angleToGoalDegrees = Constants.limelightConstants.MountingAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceFromLimelightToGoalInches = (Constants.limelightConstants.goalHeightInches - Constants.limelightConstants.limelightHeightInches) / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }

}
