// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.autonomous.AutoRoutines;
import frc.robot.subsystems.*;

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
    public final climberOLD climberOLD;
    public final Climber climber;
    public final LED led;

    // The driver's controller
    public final CommandXboxController controller0 = new CommandXboxController(0);
    public final CommandXboxController controller1 = new CommandXboxController(1);

    private static AutoRoutines autoMode;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain = new DriveSubsystem();
        superStructure = new SuperStructure();
        autoMode = new AutoRoutines(this);
        climberOLD = new climberOLD();
        climber = new Climber();
        led = new LED(1, 5);

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
        driverController();
        operatorController();
    }

    private void driverController() {
        drivetrain.setDefaultCommand(
                Commands.run(
                        () -> drivetrain.driveRobot(
                                controller0.getLeftY(),
                                controller0.getLeftX(),
                                controller0.getRightX(),
                                // RightTrigger = boost | LeftTrigger = fast
                                controller0.getRightTriggerAxis() > 0.2 ?  1 :
                                        controller0.getLeftTriggerAxis()  > 0.2 ? -1 : 0
                        ), drivetrain));
        controller0.b().onTrue(
                Commands.runOnce(
                        () -> {
                                drivetrain.zeroHeading();
                                Pose2d newBotPose = new Pose2d(drivetrain.getPose().getTranslation(), new Rotation2d(0));
                                drivetrain.resetOdometry(newBotPose);
                                Robot.m_field.setRobotPose(newBotPose);
                        }
                )
        );

        controller0.rightBumper().whileTrue(
                superStructure.bringUpRotator()
        );

       controller0.povUp().whileTrue(
            Commands.runEnd(
                () -> {
                    drivetrain.alignToTarget(Robot.hubPose);
                }, 
                () -> {
                    drivetrain.drive(0, 0, 0, true);
                }, drivetrain)
       
        );
    }

    private void operatorController() {
        // Intake fuel from intake and intake2
        controller1.leftTrigger().whileTrue(
                superStructure.runIntake()
                        .alongWith(superStructure.runLoader())
                        .alongWith(superStructure.runIntake2())
        );

        // Reject Intake2, prevents jamming issue previously encountered
        controller1.leftBumper().whileTrue(
                superStructure.rejectIntake2()
        );

        controller1.rightTrigger().whileTrue(
                Robot.checkAndRunGun(superStructure, false)
        );
        controller1.rightTrigger().onTrue(Commands.runOnce(() -> {
               Robot.loadingFuel = true;
                double now = Timer.getFPGATimestamp();
                if (now - Robot.lastTriggerPressTime < Robot.DOUBLE_PRESS_WINDOW) {
                        Robot.crippleMode = !Robot.crippleMode;
                }
                Robot.lastTriggerPressTime = now;
        }));
        controller1.rightTrigger().onFalse(Commands.runOnce(() -> Robot.loadingFuel = false));

        controller1.povDown().whileTrue(
                Commands.runOnce(
                        () -> {
                            Robot.useAutoHood = !Robot.useAutoHood;
                        }
                )
        );

        controller1.x().whileTrue(
//       superStructure.primeFlywheel(3025) // rpms lag/drop down to about 2750
//                superStructure.primeFlywheel(Robot.targetFlywheelRPM)
                superStructure.primeFlywheel(4000)
        );

        // Long distance
        controller1.y().whileTrue(
                superStructure.longDistance()
        );

        // Mid-distance
        controller1.b().whileTrue(
                superStructure.midDistance()
        );
        // Short distance
        controller1.a().whileTrue(
                superStructure.shortDistance()
        );

        // Dead man button to bring up rotator while button is pressed
        controller1.povUp().whileTrue(
                superStructure.bringUpRotator()
        );

//        controller1.povRight().whileTrue(
//                Commands.startEnd(
//                        () -> {
//                            climber.setSosftLimitsEnabled(false);
//                            climber.crawlUp();
//                        },
//                        () -> {
//                            climber.stopExtender();
//                            climber.setSoftLimitsEnabled(true);
//                        },
//                        climber
//                )
//        );
//
//        controller1.povLeft().whileTrue(
//                Commands.startEnd(
//                        () -> {
//                            climber.setSoftLimitsEnabled(false);
//                            climber.crawlDown();
//                        },
//                        () -> {
//                            climber.stopExtender();
//                            climber.setSoftLimitsEnabled(true);
//                        },
//                        climber
//                )
//        );
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

    public AutoRoutines getAutoRoutines() {
        return autoMode;
    }

    public boolean isHubActive() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) {
            return false;
        }

        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }

        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) {
            return true;
        }

        boolean redInactiveFirst = false;
        if (gameData.charAt(0) == 'R') {
            redInactiveFirst = true;
        } else {
            return true;
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> false;
            case Blue -> true;
        };

        if (matchTime > 130) {
            return true;
        } else if (matchTime > 105) {
            return shift1Active;
        } else if (matchTime > 80) {
            return !shift1Active;
        } else if (matchTime > 55) {
            return shift1Active;
        } else if (matchTime > 30) {
            return !shift1Active;
        } else {
            return true;
        }
    }
}
