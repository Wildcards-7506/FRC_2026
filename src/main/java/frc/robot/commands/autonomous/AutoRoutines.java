// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.SuperStructureConstants;

@SuppressWarnings("unused")
public final class AutoRoutines {

  private HashMap<String, Command> eventMap;
  private final AutoBuilder autoBuilder = new AutoBuilder();
  private RobotContainer robotContainer;

  // Autonomous selector on dashboard
  private final SendableChooser<Command> autoChooser;
    
  // Load the RobotConfig from the GUI settings.
  RobotConfig config;
  
  public AutoRoutines(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    setMarkers();

    try{
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
    SmartDashboard.putData("Auto Chooser",autoChooser);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) 
    -> Robot.m_field.getObject("path").setPoses(poses));
  }

  private void setMarkers() {
    //Registers commands to run in autonomous. The Pathplanner application can take these
    //pre-defined commands and place them at specific points while moving.
    NamedCommands.registerCommand("Hood Short", this.robotContainer.superStructure.shortDistance());
    NamedCommands.registerCommand("Hood Mid", this.robotContainer.superStructure.midDistance());
    NamedCommands.registerCommand("Hood Long", this.robotContainer.superStructure.longDistance());
    NamedCommands.registerCommand("Intake", 
        this.robotContainer.superStructure.runIntake()
        .alongWith(this.robotContainer.superStructure.runLoader())
        .alongWith(this.robotContainer.superStructure.runIntake2()));
    NamedCommands.registerCommand("Fire", 
        this.robotContainer.superStructure.runIntake()
        .alongWith(this.robotContainer.superStructure.rejectLoader())
        .alongWith(this.robotContainer.superStructure.runIntake2()));
    NamedCommands.registerCommand("Prime Flywheel", 
        this.robotContainer.superStructure.primeFlywheel(Constants.SuperStructureConstants.hoodMidDistance));
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