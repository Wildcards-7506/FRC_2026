// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.LimelightHelpers;
import jdk.dynalink.linker.ConversionComparator;

import javax.swing.text.html.Option;
import java.util.Optional;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private AprilTagFieldLayout fieldLayout;
  
  public static RobotContainer m_robotContainer;
  public static Field2d m_field;
  public static Limelight limelight = new Limelight();

  public static double yaw = 0.0;
  public static double tagDistance = 0.0;
  public static Pose2d targetPose = new Pose2d(0, 0, new Rotation2d());

  public static double xSpeed = 0.0;
  public static double ySpeed = 0.0;
  public static double thetaSpeed = 0.0;

  public static double speed = 0.0;
  public static double testZDistance = 0.0;
  public static double testXDistance = 0.0;

  double currentHeading = 0.0;
  double targetHeading = 0.0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_field = new Field2d();
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    SmartDashboard.putData(m_field);
    SmartDashboard.putNumber("pidp", 0.002);
//    SmartDashboard.putNumber("pid2", -1);
    SmartDashboard.putNumber("pidd", 0.01);
    m_robotContainer.drivetrain.resetOdometry(m_robotContainer.drivetrain.getPose());
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    // LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    // if(mt1.tagCount != 0 && mt1 != null){
    //   m_robotContainer.drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5,0.5,0.5));
    //   m_robotContainer.drivetrain.m_poseEstimator.addVisionMeasurement(
    //         mt1.pose,
    //         mt1.timestampSeconds);
    // }

    LimelightHelpers.SetRobotOrientation("limelight", m_robotContainer.drivetrain.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    boolean doRejectUpdate = Math.abs(m_robotContainer.drivetrain.m_gyro.getRate()) > 360;
    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      // 0.5,0.5,0.5 original
      m_robotContainer.drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      m_robotContainer.drivetrain.m_poseEstimator.addVisionMeasurement(
              mt2.pose,
              mt2.timestampSeconds);
    }

    CommandScheduler.getInstance().run();

//    Pose2d robotPos = m_robotContainer.drivetrain.getPose();
//    double robotPosX = robotPos.getX();
//    double robotPosY = robotPos.getY();
//
//    m_field.setRobotPose(robotPos);
//
//    double target3dX = 0;
//    double target3dY = 0;
//    double target3dZ = 0;
//    Optional<Pose3d> target = fieldLayout.getTagPose(26);
//    if (target != null && target.isPresent()) {
//      Pose3d target3d = target.get();
//
//      Pose2d hardCodeTargetPose = target.get().toPose2d();
//
//      Pose2d fixedTargetPos = hardCodeTargetPose.times(39.37008);

//      target3dX = Units.metersToInches(target3d.getX());
//      target3dY = Units.metersToInches(target3d.getY());
//      target3dZ = Units.metersToInches(target3d.getZ());

//      Pose2d targetPoseWithOffset = hardCodeTargetPose.transformBy(
//              new Transform2d(Units.inchesToMeters(Constants.limelightConstants.targetDistance), 0, new Rotation2d())
//      );

      // double targetX = hardCodeTargetPose.getX();
      // double targetY = hardCodeTargetPose.getY();

      // Rotation2d angleToTarget = new Rotation2d(
      //           targetY - robotPosY,
      //           targetX - robotPosX
      // );

//      targetPose = new Pose2d(
//              hardCodeTargetPose.getTranslation(),
//              angleToTarget
//      );

//      targetPose = fixedTargetPos;
//    }

//     Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");

//     double x_inches = pose.getX() * 39.37008;
//     double y_inches = pose.getY() * 39.37008;
//     double z_inches = pose.getZ() * 39.37008;

//     yaw = Math.toDegrees(pose.getRotation().getY());
//     tagDistance = Math.sqrt(Math.pow(x_inches, 2) + Math.pow(z_inches, 2));

//     double constSpeed = 0.01;

//     testZDistance = z_inches - Constants.limelightConstants.targetDistance;
//     testXDistance = x_inches;

//     if (testXDistance > 3) {
//       xSpeed = constSpeed;
//     }else if (testXDistance < -3) {
//       xSpeed = -constSpeed;
//     }else {
//       xSpeed = 0.0;
//     }

//     if (testZDistance > 3) {
//       ySpeed = -constSpeed;
//     }else if (testZDistance < -3) {
//       ySpeed = constSpeed;
//     }else {
//       ySpeed = 0.0;
//     }


    SmartDashboard.putNumber("Flywheel RPM", m_robotContainer.superStructure.getRPM());
    SmartDashboard.putNumber("GyroHeading", m_robotContainer.drivetrain.getHeading());
    SmartDashboard.putNumber("GyroAngleZ", m_robotContainer.drivetrain.m_gyro.getAngle());
    SmartDashboard.putNumber("LX", m_robotContainer.controller0.getLeftX());
    SmartDashboard.putNumber("LY", m_robotContainer.controller0.getLeftY());
    SmartDashboard.putNumber("RX", m_robotContainer.controller0.getRightX());
    SmartDashboard.putBoolean("FieldRelative", m_robotContainer.drivetrain.isFieldRel);

    SmartDashboard.putNumber("Robot Yaw", yaw);
    SmartDashboard.putNumber("Flat Plane Tag Distance", tagDistance);
    SmartDashboard.putNumber("TX", LimelightHelpers.getTX("limelight"));
    SmartDashboard.putNumber("Yaw PID", (yaw / 11.5) * Constants.limelightConstants.yawOutputMultiplier);
    SmartDashboard.putNumber("Hood degrees", m_robotContainer.superStructure.getHoodPos());

//    SmartDashboard.putBoolean("Target present", target != null && target.isPresent());
//    SmartDashboard.putNumber("Target Pose X", targetPose.getX());
//    SmartDashboard.putNumber("Target Pose Y", targetPose.getY());
//    SmartDashboard.putNumber("Robot Pose X", robotPos.getX());
//    SmartDashboard.putNumber("Robot Pose Y", robotPos.getY());
//
//    SmartDashboard.putNumber("3d Robot Pose X", target3dX);
//    SmartDashboard.putNumber("3d Robot Pose Y", target3dY);
//    SmartDashboard.putNumber("3d Robot Pose Z", target3dZ);

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.superStructure.setRotatorPos(Constants.SuperStructureConstants.rotatorMax);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
