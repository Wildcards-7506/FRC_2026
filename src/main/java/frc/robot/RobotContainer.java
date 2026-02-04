// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem drivetrain = new DriveSubsystem();

  // The driver's controller
    public final CommandXboxController controller0 = new CommandXboxController(0);
    public final CommandXboxController controller1 = new CommandXboxController(1);
    public static SlewRateLimiter xLimiter = new SlewRateLimiter(1);
    public static SlewRateLimiter yLimiter = new SlewRateLimiter(1);
    boolean boostToggle = false;
    double fullTurnSpeed = 0.30;
    double fullDriveSpeed = 0.20;
    double fineTurnSpeed = 0.2; // current default state
    double fineDriveSpeed = 0.1; // current default state
    double boostDriveSpeed = 0.5;
    double boostTurnSpeed = 0.25;

  public final Shooter shooter;
  public final ShooterCommands shooterCommands;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    drivetrain.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> Drivespeed(),
            drivetrain));

    shooter = new Shooter();
    shooterCommands = new ShooterCommands(shooter);
  }
  void Drivespeed(){

    double forwardspeed = controller0.getLeftY()*((controller0.getRightTriggerAxis()>0.2)?boostDriveSpeed:fullDriveSpeed);
    double strafingSpeed = controller0.getLeftX()*((controller0.getRightTriggerAxis()>0.2)?boostDriveSpeed:fullDriveSpeed);
    double rotationSpeed = controller0.getRightX()*((controller0.getRightTriggerAxis()>0.2)?boostTurnSpeed:fullTurnSpeed);

    forwardspeed = xLimiter.calculate(forwardspeed);
    strafingSpeed = xLimiter.calculate(strafingSpeed);

    drivetrain.drive(
                -MathUtil.applyDeadband(forwardspeed, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(strafingSpeed, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(rotationSpeed, OIConstants.kDriveDeadband),
                true);
  }
    // private double getDriveSpeed(double input) {
    //     return this.boostToggle ?
    //             PlayerConfigs.boostDriveSpeed * input :
    //             PlayerConfigs.fullDriveSpeed * input;
    // }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    controller0.b().onTrue(
      Commands.runOnce(
          () -> drivetrain.zeroHeading()
      )
    );
    
    controller1.leftTrigger().onTrue(
      Commands.runOnce(
        () -> {
          shooter.setIntakeVoltage(12);
          shooter.setLoaderVoltage(12);
        }
      )
    );
    controller1.leftTrigger().onFalse(
      Commands.runOnce(
        () -> {
          shooter.setIntakeVoltage(0);
          shooter.setLoaderVoltage(0);
        }
      )
    );
    
    controller1.rightTrigger().onTrue(
      Commands.runOnce(
        () -> {
          // shooter.setFlywheelVoltage(12);
          shooter.setFlywheelRPM(5000);
        }
      )
    );
    controller1.rightTrigger().onFalse(
      Commands.runOnce(
        () -> { 
          // shooter.setFlywheelVoltage(0);
          shooter.setFlywheelRPM(0);
        }
      )
    );

    controller1.a().onTrue (
      Commands.runOnce (
        () -> {
          shooter.setIntakeVoltage(12);
          shooter.setLoaderVoltage(-12);
        }
      )
    );
    controller1.a().onFalse (
      Commands.runOnce (
        () -> {
          shooter.setIntakeVoltage(0);
          shooter.setLoaderVoltage(0);
        }
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        drivetrain::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drivetrain::setModuleStates,
        drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false));
  }
}
