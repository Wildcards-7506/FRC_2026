// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.concurrent.TransferQueue;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.autonomous.commands.GunCommand;
import frc.robot.commands.autonomous.commands.IntakeCommand;
import frc.robot.commands.autonomous.commands.RotatorDownCommand;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.subsystems.Agitator;

@SuppressWarnings("unused")
public final class AutoRoutines {

    private HashMap<String, Command> eventMap;
    private final AutoBuilder autoBuilder = new AutoBuilder();
    private RobotContainer robotContainer;

    // Autonomous selector on dashboard
    private final SendableChooser<Command> autoChooser;

    // Load the RobotConfig from the GUI settings.
    RobotConfig config;

    private RepeatCommand gunRepeatingCommand = null;

    public AutoRoutines(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        setMarkers();

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this.robotContainer.drivetrain::getPose, // Robot pose supplier
                this.robotContainer.drivetrain::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this.robotContainer.drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> this.robotContainer.drivetrain.driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent())
                        return alliance.get() == DriverStation.Alliance.Red;
                    return false;
                },
                this.robotContainer.drivetrain // Reference to this subsystem to set requirements
        );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses)
                -> Robot.m_field.getObject("path").setPoses(poses));
    }

    private void setMarkers() {
        //Registers commands to run in autonomous. The Pathplanner application can take these
        //pre-defined commands and place them at specific points while moving.
        NamedCommands.registerCommand("intake on", this.robotContainer.superStructure.enableIntakes()); // Use this as a guide for complex sequences
        NamedCommands.registerCommand("intake off", this.robotContainer.superStructure.disableIntakes()); // Use this as a guide for complex sequences
        NamedCommands.registerCommand("gun on", this.robotContainer.superStructure.enableFlywheel(Constants.ShooterConstants.flywheelRPM)); // Use this as a guide for complex sequences
        //  NamedCommands.registerCommand("gun on", runGun()); // Use this as a guide for complex sequences
//  NamedCommands.registerCommand("shoot", new GunCommand()); // Use this as a guide for complex sequences
        NamedCommands.registerCommand("shoot",
                Commands.waitUntil(() -> Robot.flywheelHitTarget)
                        .andThen(this.robotContainer.superStructure.enableRejectLoader())
                        .andThen(this.robotContainer.superStructure.enableIntake())
                        .andThen(this.robotContainer.superStructure.enableIntake2())
                        .andThen(Agitator.enableRight())
                        .andThen(Agitator.enableLeft())
        ); // Use this as a guide for complex sequences
        NamedCommands.registerCommand("gun off", gunOff()); // Use this as a guide for complex sequences
//   NamedCommands.registerCommand("gun on",  this.robotContainer.superStructure.enableFlywheel(Constants.ShooterConstants.flywheelRPM)); // Use this as a guide for complex sequences

        NamedCommands.registerCommand("start booty bump", Commands.runOnce(() -> {
            Robot.bootyBumpEnabled = true;
//            CommandScheduler.getInstance().schedule(this.robotContainer.superStructure.bootyBumpCommand(150, 680));
            CommandScheduler.getInstance().schedule(this.robotContainer.superStructure.bootyBumpCommand(150, 1200));
        }));

        NamedCommands.registerCommand("end booty bump", Commands.runOnce(() -> {
            SmartDashboard.putBoolean("Should I stop?", true);
            System.out.println("Ended booty bump.");
//            Robot.bootyBumpEnabled = false;
            Robot.bootyBumpEnabled = false;
            CommandScheduler.getInstance().schedule(this.robotContainer.superStructure.disableBootyBump());
        }));

        NamedCommands.registerCommand("Intake Down", new RotatorDownCommand(this.robotContainer)); // Use this as a guide for complex sequences
        NamedCommands.registerCommand("Gun And Load 3", Robot.primeAndRunGun(this.robotContainer.superStructure).withTimeout(3)); // implement this
        NamedCommands.registerCommand("Gun And Load 4", Robot.primeAndRunGun(this.robotContainer.superStructure).withTimeout(4)); // implement this
        NamedCommands.registerCommand("Gun And Load 5", Robot.primeAndRunGun(this.robotContainer.superStructure).withTimeout(5)); // implement this
        NamedCommands.registerCommand("Gun And Load 8", Robot.primeAndRunGun(this.robotContainer.superStructure).withTimeout(8)); // implement this
        NamedCommands.registerCommand("Gun And Load 25", Robot.primeAndRunGun(this.robotContainer.superStructure).withTimeout(25)); // implement this
        NamedCommands.registerCommand("Intake", new IntakeCommand(2)); // Use this as a guide
        NamedCommands.registerCommand("Intake 1", new IntakeCommand(2).withTimeout(1)); // Use this as a guide
        NamedCommands.registerCommand("Intake 2", new IntakeCommand(2).withTimeout(2)); // Use this as a guide
        NamedCommands.registerCommand("Intake 3", new IntakeCommand(2).withTimeout(3)); // Use this as a guide
        NamedCommands.registerCommand("Intake 4", new IntakeCommand(2).withTimeout(4)); // Use this as a guide
        NamedCommands.registerCommand("Intake 5", new IntakeCommand(2).withTimeout(5)); // Use this as a guide
        NamedCommands.registerCommand("Intake 6.5", new IntakeCommand(2).withTimeout(6.5)); // Use this as a guide
//    NamedCommands.registerCommand("Intake Down", this.robotContainer.superStructure.setRotator(SuperStructureConstants.rotatorMax)); // use this as a guide for simple actions
        // NamedCommands.registerCommand("Prime Flywheel", ); // implement this
    }

    public Command gunOff() {
        return this.robotContainer.superStructure.disableIntakes()
                .alongWith(this.robotContainer.superStructure.disableFlywheel())
                .andThen(Agitator.stopMotorsCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void resetAutoHeading() {
        this.robotContainer.drivetrain.zeroHeading();
    }
}