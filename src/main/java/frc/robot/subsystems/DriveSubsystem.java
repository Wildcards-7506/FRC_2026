// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {
  // public double testCounter = 0; // Used to test if command chaining is correct, e.g. runOnce does runOnce, runEnd does run again until end
  Pose2d savedPose = null;
  public boolean isFieldRel = true;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  // TODO: Put gyro back to private after testing orientation
  public AHRS m_gyro = new AHRS(AHRS.NavXComType.kUSB1);


  // Pose estimator class for tracking robot pose
  public final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  private static SlewRateLimiter xLimiter = new SlewRateLimiter(1);
  private static SlewRateLimiter yLimiter = new SlewRateLimiter(1);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_poseEstimator.update(
      m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
      m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = -xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
            m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Method to drive the robot
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rotSpeed      Angular rate of the robot.
   * @param speedMode         -1 = slow, 0 = normal, 1 = speedMode
   */
  public void driveRobot(double xSpeed, double ySpeed, double rotSpeed, int speedMode) {
    double driveSpeed = switch (speedMode) {
      case 2  -> DriveConstants.boostDriveSpeed;  // LT  — FULL BOOST
      case 1  -> DriveConstants.fullDriveSpeed;   // RT  — Full
      default -> DriveConstants.fineDriveSpeed;   // none — Fine
    };

    double turnScale = switch (speedMode) {
      case 2  -> DriveConstants.boostTurnSpeed;
      case 1  -> DriveConstants.fullTurnSpeed;
      default -> DriveConstants.fineTurnSpeed;
    };

    double forwardSpeed  = xSpeed * driveSpeed;
    double strafingSpeed = ySpeed * driveSpeed;
    double rotationSpeed = rotSpeed * turnScale;

    forwardSpeed  = yLimiter.calculate(forwardSpeed);
    strafingSpeed = xLimiter.calculate(strafingSpeed);

    drive(
            MathUtil.applyDeadband(strafingSpeed, IOConstants.kDriveDeadband),
            -MathUtil.applyDeadband(forwardSpeed,  IOConstants.kDriveDeadband),
            -MathUtil.applyDeadband(rotationSpeed, IOConstants.kDriveDeadband),
            isFieldRel);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  // public Command alignToTarget(Supplier<Pose2d> targetSupplier) {
  public void alignToTarget(Pose2d targetSupplier) {
    double rotSpeed = 0.4;

    Pose2d botPose = getPose();
    // Pose2d target = targetSupplier.get();
    Pose2d target = targetSupplier;

    double dX = target.getX() - botPose.getX();
    double dY = target.getY() - botPose.getY();
    double targetDeg = Math.toDegrees(Math.atan2(dY, dX));

    SmartDashboard.putNumber("BotHeading", getHeading());
    SmartDashboard.putNumber("HeadingToTarget", targetDeg);
    SmartDashboard.putNumber("HeadingToTarget2", Robot. angleToHub.getDegrees());

    double err = MathUtil.inputModulus(targetDeg - getHeading(), -180, 180);

    err = err / 50;

    double speed = filterValue(err, -rotSpeed, rotSpeed);

    drive(0, 0, speed, true);
  }

  public void savePose() {
    this.savedPose = this.getPose();
  }

  // public Command alignToTarget(Supplier<Pose2d> targetSupplier) {
  public void alignToTargetHoldPose(Pose2d targetSupplier) {
    double rotSpeed = 0.4;

    Pose2d botPose = getPose();
    // Pose2d target = targetSupplier.get();
    Pose2d target = targetSupplier;

    double dX = target.getX() - botPose.getX();
    double dY = target.getY() - botPose.getY();
    double targetDeg = Math.toDegrees(Math.atan2(dY, dX));

    SmartDashboard.putNumber("BotHeading", getHeading());
    SmartDashboard.putNumber("HeadingToTarget", targetDeg);
    SmartDashboard.putNumber("HeadingToTarget2", Robot.angleToHub.getDegrees());
    
    double err = MathUtil.inputModulus(targetDeg - getHeading(), -180, 180);
    
    err = err / 50;
    
    rotSpeed = filterValue(err, -rotSpeed, rotSpeed);
    
    Pose2d currPose = getPose();

    double maxSpeed = 0.3;
    
    double xSpeed = Math.min(maxSpeed, Math.max(-maxSpeed, this.savedPose.getX() - currPose.getX()));
    double ySpeed = Math.min(maxSpeed, Math.max(-maxSpeed, this.savedPose.getY() - currPose.getY()));
    
    SmartDashboard.putNumber("Hold X Input", xSpeed);
    SmartDashboard.putNumber("Hold Y Input", ySpeed);
    SmartDashboard.putNumber("Hold X CurrPose", currPose.getX());
    SmartDashboard.putNumber("Hold Y CurrPose", currPose.getY());
    SmartDashboard.putNumber("Hold X SavedPose", this.savedPose.getX());
    SmartDashboard.putNumber("Hold Y SavedPose", this.savedPose.getY());
    drive(-ySpeed, xSpeed, rotSpeed, true);
  }

    /**
     * Returns the max if the setpoint is above it, or the hard deck if the setpoint is below it.
     * Otherwise, returns the setpoint.
     * 
     * @param value The desired state of the crane.
     * @param min The hard deck of the crane.
     */
    public double filterValue(double value, double min, double max) {
        if(value < min)
            value = min;
        if(value > max)
            value = max;
        return value;
    }
  
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

    //@Logged(name = "Chassis Speed")
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds
      (m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    );
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}