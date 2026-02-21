// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.utils.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.lang.reflect.Field;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem drivetrain;
  public final SuperStructure superStructure;

  // The driver's controller
    public final CommandXboxController controller0 = new CommandXboxController(0);
    public final CommandXboxController controller1 = new CommandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain = new DriveSubsystem();
    superStructure = new SuperStructure();

    // Configure the button bindings
    configureButtonBindings();    
  }


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

    drivetrain.setDefaultCommand(
    Commands.runOnce(
      () -> drivetrain.driveRobot(
        controller0.getLeftX(),
        -controller0.getLeftY(),
        -controller0.getRightX(),
        controller0.getRightTriggerAxis() < 0.2
      ), drivetrain));

    controller0.b().onTrue(
      Commands.runOnce(
          () -> drivetrain.zeroHeading()
      )
    );
    
    //Run intake and loader inwards
    controller1.leftTrigger().whileTrue(
      superStructure.runIntake().alongWith(superStructure.runLoader()).alongWith(superStructure.runIntake2())
    );

    controller1.povLeft().whileTrue(
      Commands.repeatingSequence(superStructure.runRotator(this))
    );

    //Spin up the shooter
    // controller1.y().whileTrue(
    //   superStructure.primeFlywheel(3200) // diffrerent button will call this with diffrerent rpm
    // );

    // controller1.b().whileTrue(
    //   superStructure.primeFlywheel()
    // );

    // controller1.a().whileTrue(
    //   superStructure.primeFlywheel()
    // );

    // controller1.rightTrigger().whileTrue(
      // superStructure.runIntake().alongWith(superStructure.rejectLoader());
    // );
    
    //Spin up the shooter
    controller1.rightTrigger().whileTrue(
//      superStructure.primeFlywheel(SmartDashboard.getNumber("RPM", 3200))
      superStructure.primeFlywheel(3200)
    );

    //run intake inwards and loader outwards, use for firing shooter
    controller1.a().whileTrue(
      superStructure.runIntake().alongWith(superStructure.rejectLoader())
    );

    controller0.leftBumper()
    .whileTrue(new RunCommand(
        () -> drivetrain.drive(
          controller0.getLeftY(),
          controller0.getLeftX(),
          LimelightHelpers.getTX("limelight") * 0.05, //Placeholder
          false
        ),

        drivetrain

      ));

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
