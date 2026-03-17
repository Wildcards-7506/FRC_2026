// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.RobotContainer.loadingFuel;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autonomous.AutoRoutines;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SuperStructure;
import frc.robot.utils.LimelightHelpers;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static RobotContainer m_robotContainer;
  public static Field2d m_field;
  private AprilTagFieldLayout fieldLayout;
  public static AutoRoutines autoMode;

  public static Limelight limelight = new Limelight();

  public static double yaw = 0.0;
  public static double tagDistance = 0.0;

  public static Pose2d targetPose = new Pose2d(0, 0, new Rotation2d());

  double currentHeading = 0.0;
  double targetHeading = 0.0;

  // Used for logging purposes, can be used for functionality but not intended
  public static double flywheelHighestRPM = 0.0;
  public static double flywheelLowestRPM = 0.0;
  public static boolean flywheelHitTarget = false; // Checks if flywheel has hit target speed at-least once within init
  public static double distanceToHub = 0.0;
  public static Rotation2d angleToHub = new Rotation2d();

  public static boolean crippleMode = false;
  public static int triggerPressCount = 0;
  public static double lastTriggerPressTime = -1;
  public static final double DOUBLE_PRESS_WINDOW = 0.25; // seconds

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
    autoMode = m_robotContainer.getAutoRoutines();
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

    CommandScheduler.getInstance().run();
    limelightUpdateOdom();

    updateHeadingToHub();

    Pose2d tagPose = fieldLayout.getTagPose(26).get().toPose2d();
    double offsetMeters = Units.inchesToMeters(Constants.limelightConstants.targetDistance);
    targetPose = tagPose.transformBy(
            new Transform2d(offsetMeters, 0, new Rotation2d())
    );

    updateFlywheelLogs();

    SmartDashboard.putBoolean("Flywheel target hit", flywheelHitTarget);
    SmartDashboard.putNumber("Flywheel Lowest", flywheelLowestRPM);
    SmartDashboard.putNumber("Flywheel Highest", flywheelHighestRPM);
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

    SmartDashboard.putNumber("Target Pose X", targetPose.getX());
    SmartDashboard.putNumber("Target Pose Y", targetPose.getY());
    SmartDashboard.putNumber("Robot Pose X", m_robotContainer.drivetrain.getPose().getX());
    SmartDashboard.putNumber("Robot Pose Y", m_robotContainer.drivetrain.getPose().getY());

    SmartDashboard.putNumber("DistanceToHub", Units.metersToInches(distanceToHub));
    SmartDashboard.putNumber("AngleToHub", angleToHub.getDegrees());

    SmartDashboard.putBoolean("Crippled?", crippleMode);
  }

  private void updateHeadingToHub() {    
    Pose2d hubPose = new Pose2d(Units.inchesToMeters(157.79), Units.inchesToMeters(158.32), new Rotation2d());

    Pose2d botPose = m_robotContainer.drivetrain.getPose();

    double dX = hubPose.getX() - botPose.getX(); // delta x
    double dY = hubPose.getY() - botPose.getY(); // delta y

    distanceToHub = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));

    angleToHub = new Rotation2d(distanceToHub == 0 ? 0 : Math.acos(dX / distanceToHub));
  }

  private void limelightUpdateOdom() {
    LimelightHelpers.SetRobotOrientation("limelight", m_robotContainer.drivetrain.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    boolean doRejectUpdate = false;
    if (Math.abs(m_robotContainer.drivetrain.m_gyro.getRate()) > 360) {
      doRejectUpdate = true;
    }
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

    m_field.setRobotPose(m_robotContainer.drivetrain.getPose());
  }

  private void updateFlywheelLogs() {
      double currentRPM = m_robotContainer.superStructure.getRPM();

      if (currentRPM > Constants.SuperStructureConstants.baseFlywheelRpm) {
          flywheelHitTarget = true;
      } else if (currentRPM < (Constants.SuperStructureConstants.baseFlywheelRpm - 2000)) {
          flywheelHitTarget = false;
      }

      if (loadingFuel && flywheelHitTarget) {
          if (currentRPM > flywheelHighestRPM) {
              flywheelHighestRPM = currentRPM;
          }

          if (flywheelLowestRPM == 0 || currentRPM < flywheelLowestRPM) {
              flywheelLowestRPM = currentRPM;
          }
      }
  }

//  public static Command checkAndRunGun(SuperStructure superStructure) {
//    return Commands.either(
//            superStructure.runIntake()
//                    .alongWith(superStructure.rejectLoader())
//                    .alongWith(superStructure.runIntake2()),
//            Commands.idle(),
//            () -> flywheelHitTarget
//    );
//  }

  public static Command checkAndRunGun(SuperStructure superStructure) {
    return Commands.either(
            superStructure.runIntake()
                    .alongWith(superStructure.rejectLoader())
                    .alongWith(superStructure.runIntake2()),

            Commands.waitUntil(() -> Robot.flywheelHitTarget)
                    .andThen(
                            superStructure.runIntake()
                                    .alongWith(superStructure.rejectLoader())
                                    .alongWith(superStructure.runIntake2())
                    ),

            () -> crippleMode
    );
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.superStructure.setIntake2Voltage(0);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    // Set robot state
    autoMode.resetAutoHeading();
//    autoMode.getAutonomousCommand().schedule();
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
    CommandScheduler.getInstance().cancelAll();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    flywheelHitTarget = false;
    flywheelHighestRPM = 0.0;
    flywheelLowestRPM = 0.0;

    m_robotContainer.superStructure.setRotatorPos(Constants.SuperStructureConstants.rotatorMax);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // m_robotContainer.superStructure.setIntake2Voltage(12);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
