// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.autonomous.AutoRoutines;
import frc.robot.commands.autonomous.commands.GunCommand;
import frc.robot.subsystems.*;
import frc.robot.utils.LimelightHelpers;

import static frc.robot.utils.LimelightHelpers.setLEDMode_ForceOff;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    public static RobotContainer m_robotContainer = new RobotContainer();
    public static Field2d m_field;
    private static AprilTagFieldLayout fieldLayout;
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
    public static Pose2d centerTagPose = new Pose2d(0, 0, new Rotation2d());
    public static Pose2d hubPose = new Pose2d(0, 0, new Rotation2d());
    //  public static PIDController rotPID = new PIDController(0.02, 0, 0.01);

    public static boolean crippleMode = false;
    public static int triggerPressCount = 0;
    public static double lastTriggerPressTime = -1;
    public static final double DOUBLE_PRESS_WINDOW = 0.25; // seconds
    public static boolean useAutoHood = true;

    public static boolean loadingFuel = false;

    public static InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap flywheelTable = new InterpolatingDoubleTreeMap();

    public static boolean bootyBumpEnabled = false;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_field = new Field2d();
        autoMode = m_robotContainer.getAutoRoutines();
        SmartDashboard.putData(m_field);
        SmartDashboard.putNumber("pidp", 0.002);
        // SmartDashboard.putNumber("pid2", -1);
        SmartDashboard.putNumber("pidd", 0.01);
        m_robotContainer.drivetrain.resetOdometry(m_robotContainer.drivetrain.getPose());
        m_robotContainer.led.periodic();
        m_robotContainer.led.setColorType(LED.ColorType.FLASH);
        m_robotContainer.led.setColors(new int[][]{{161, 33, 38}, {255, 255, 255}, {0, 0, 0}});
        setLEDMode_ForceOff("limelight");
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.

        if (fieldLayout == null)
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        limelightUpdateOdom();
        setLEDMode_ForceOff("limelight");

        Optional<Alliance> alliances = DriverStation.getAlliance();
        boolean onBlueAlliance = alliances.isPresent() ? alliances.get() == DriverStation.Alliance.Blue : true;
        // double offsetToHubCenter = 0.0;
//        m_robotContainer.led.setColorType(LED.ColorType.ALLIANCE_FLOW);

        if (fieldLayout != null) {
            centerTagPose = fieldLayout.getTagPose(onBlueAlliance
                            ? Constants.limelightConstants.blueAlianceCeterTagNum
                            : Constants.limelightConstants.redAlianceCeterTagNum)
                    .get().toPose2d();
            // offsetToHubCenter = Units.inchesToMeters(23.5) * (onBlueAlliance ? 1 : -1);
        }

        double offsetToHubCenter = Units.inchesToMeters(-23.5); // eyeballed at 2 feet from tag 26 to hub center, x direction wpi
        hubPose = centerTagPose.transformBy(new Transform2d(offsetToHubCenter, 0, new Rotation2d()));
//        hubPose = centerTagPose.transformBy(new Transform2d(-offsetToHubCenter, 0, new Rotation2d()));

        SmartDashboard.putNumber("Tag Pose Offset X", offsetToHubCenter);
        SmartDashboard.putNumber("Center Tag Pose X", centerTagPose.getX());
        SmartDashboard.putNumber("Hub Pose X", hubPose.getX());

        CommandScheduler.getInstance().run();

        // blue, field coordinates

        updateHeadingToHub();

        updateFlywheelLogs();

        if (useAutoHood) {
//             Robot.setRPMForCurrentDistance();
            Robot.setHoodForCurrentDistance();
            Robot.setFlywheelForCurrentDistance();
        } else {

        }

        m_field.getObject("currentTarget").setPose(hubPose);

        if (!useAutoHood && !loadingFuel) {
            m_robotContainer.led.setColorType(LED.ColorType.SOLID);
            m_robotContainer.led.setColor(128, 0, 128);
        }

        if (crippleMode) {
            m_robotContainer.led.setColorType(LED.ColorType.FLASH);
            m_robotContainer.led.setColor(255, 140, 0);
        }

        if (DriverStation.getMatchNumber() > 0 && !m_robotContainer.isHubActive()) {
            m_robotContainer.led.setColorType(LED.ColorType.SOLID);
            m_robotContainer.led.setColor(255, 0, 0);
        }

        putSmartDashboardValues();
    }

    private void putSmartDashboardValues() {
        SmartDashboard.putNumber("TagPose X", centerTagPose.getX());
        SmartDashboard.putNumber("TagPose Y", centerTagPose.getY());
        SmartDashboard.putNumber("TagPose Yaw", centerTagPose.getRotation().getDegrees());
        SmartDashboard.putNumber("HubPose X", hubPose.getX());
        SmartDashboard.putNumber("HubPose Y", hubPose.getY());
        SmartDashboard.putNumber("HubPose Yaw", hubPose.getRotation().getDegrees());

        SmartDashboard.putBoolean("Use auto hood", useAutoHood);
        SmartDashboard.putBoolean("Field layout here?", fieldLayout != null);

        SmartDashboard.putBoolean("Flywheel target hit", flywheelHitTarget);
        SmartDashboard.putNumber("Flywheel Lowest", flywheelLowestRPM);
        SmartDashboard.putNumber("Flywheel Highest", flywheelHighestRPM);
        SmartDashboard.putNumber("Flywheel RPM", m_robotContainer.superStructure.getRPM());
        SmartDashboard.putNumber("Flywheel Target", Constants.ShooterConstants.flywheelRPM);

        SmartDashboard.putNumber("Hood target", m_robotContainer.superStructure.getHoodTarget());

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
        SmartDashboard.putNumber("Field Robot Pose X", m_field.getRobotPose().getX());
        SmartDashboard.putNumber("Field Robot Pose Y", m_field.getRobotPose().getY());

        SmartDashboard.putNumber("DistanceToHub", Units.metersToInches(distanceToHub));
        SmartDashboard.putNumber("DistanceToHubMeters", distanceToHub);
        SmartDashboard.putNumber("AngleToHub", angleToHub.getDegrees());

        SmartDashboard.putBoolean("Crippled?", crippleMode);

        SmartDashboard.putNumber("Hub X", hubPose.getX());
        SmartDashboard.putNumber("Hub Y", hubPose.getY());

        SmartDashboard.putBoolean("Stop Gun?", GunCommand.stopGun);
//        SmartDashboard.putNumber("Loader RPM", m_robotContainer.superStructure.getAgitatorRPM());
    }

    private void updateHeadingToHub() {

        Pose2d botPose = m_robotContainer.drivetrain.getPose();

        double dX = hubPose.getX() - botPose.getX(); // delta x
        double dY = hubPose.getY() - botPose.getY(); // delta y

        distanceToHub = Math.sqrt(Math.pow(dX, 2) + Math.pow(dY, 2));
        angleToHub = new Rotation2d(distanceToHub == 0 ? 0 : Math.acos(dX / distanceToHub));
    }

    private void limelightUpdateOdom() {
        LimelightHelpers.SetRobotOrientation("limelight",
                m_robotContainer.drivetrain.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
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

        if (currentRPM > Constants.ShooterConstants.flywheelRPM - 200) {
            flywheelHitTarget = true;
            m_robotContainer.led.setColor(0, 255, 0);
            m_robotContainer.led.setColorType(LED.ColorType.FLASH);
        } else if (currentRPM < (Constants.ShooterConstants.flywheelRPM - 2000)) {
            flywheelHitTarget = false;
        }

        if (loadingFuel && flywheelHitTarget) {
            if (currentRPM > flywheelHighestRPM) {
                flywheelHighestRPM = currentRPM;
            } else {
                m_robotContainer.led.setColor(0, 255, 0);
                m_robotContainer.led.setColorType(LED.ColorType.FLASH);
            }

            if (flywheelLowestRPM == 0 || currentRPM < flywheelLowestRPM) {
                flywheelLowestRPM = currentRPM;
            }
        }

        if (loadingFuel && m_robotContainer.superStructure.intake.getEncoder().getVelocity() < 20) {
            m_robotContainer.led.setColor(255, 0, 0);
            m_robotContainer.led.setColorType(LED.ColorType.FLASH);
        }
    }

    public static Command checkAndRunGun(SuperStructure superStructure, Boolean useAuto) {
        return Commands.either(
                superStructure.runIntake()
                        .alongWith(useAuto ? superStructure.rejectLoaderAuto() : superStructure.rejectLoader())
                        .alongWith(superStructure.runIntake2()),
                Commands.waitUntil(() -> Robot.flywheelHitTarget)
                        .andThen(
                                superStructure.runIntake()
                                        .alongWith(useAuto ? superStructure.rejectLoaderAuto() : superStructure.rejectLoader())
                                        .alongWith(superStructure.runIntake2()))
                        .alongWith(Agitator.runRight())
                        .alongWith(Agitator.runLeft()),
                () -> crippleMode);
    }

    public static Command primeAndRunGun(SuperStructure superStructure) {
//        return superStructure.primeFlywheel(4000).alongWith(checkAndRunGun(superStructure, true));
        return superStructure.primeFlywheel(Constants.ShooterConstants.flywheelRPM).alongWith(checkAndRunGun(superStructure, true));
    }

    public static void setHoodForCurrentDistance() {
        hoodTable.put(215.0, 7.277);
        hoodTable.put(200.0, 5.665);
        hoodTable.put(190.0, 3.614);
        hoodTable.put(180.0, 1.172);
        hoodTable.put(170.0, -0.88);
        hoodTable.put(160.0, -2.54);
        hoodTable.put(150.0, -3.663);
        hoodTable.put(140.0, -4.786);
        hoodTable.put(130.0, -6.642);
        m_robotContainer.superStructure.setHoodPos(hoodTable.get(Units.metersToInches(distanceToHub)));
    }

    public static void setFlywheelForCurrentDistance() {
        // PRE STATE
//        flywheelTable.put(215.0, 4500.0);
//        flywheelTable.put(120.0, 4500.0);
//        flywheelTable.put(110.0, 4300.0);
//        flywheelTable.put(100.0, 4100.0);
//        flywheelTable.put(90.0, 3900.0);
//        flywheelTable.put(80.0, 3700.0);
//        flywheelTable.put(70.0, 3500.0);
//        flywheelTable.put(60.0, 3300.0);

        // POST STATE
        flywheelTable.put(215.0, 4500.0);
        flywheelTable.put(120.0, 4500.0);
        flywheelTable.put(110.0, 4375.0);
        flywheelTable.put(100.0, 4175.0);
        flywheelTable.put(90.0, 3975.0);
        flywheelTable.put(80.0, 3800.0);
        flywheelTable.put(70.0, 3600.0);
        flywheelTable.put(60.0, 3300.0);
        Constants.ShooterConstants.flywheelRPM = flywheelTable.get(Units.metersToInches(Robot.distanceToHub));
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        m_robotContainer.superStructure.setIntake2Voltage(0);
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.led.setColorType(LED.ColorType.ALLIANCE_FLOW);
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();

        // Set robot state
        autoMode.resetAutoHeading();
        m_robotContainer.led.setColorType(LED.ColorType.STREAMER);
        CommandScheduler.getInstance().schedule(autoMode.getAutonomousCommand());
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        if (!loadingFuel) {
            m_robotContainer.led.setColorType(LED.ColorType.RAINBOW);
        }
        updateFlywheelLogs();
        // CommandScheduler.getInstance().run();
    }

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
        m_robotContainer.led.setColorType(LED.ColorType.STREAMER);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        // m_robotContainer.superStructure.setIntake2Voltage(12);
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
