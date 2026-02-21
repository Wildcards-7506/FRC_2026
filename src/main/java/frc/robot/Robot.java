// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.LimelightHelpers;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Field2d m_field;

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
    SmartDashboard.putData(m_field);
    SmartDashboard.putNumber("pidp", 0.002);
//    SmartDashboard.putNumber("pid2", -1);
    SmartDashboard.putNumber("pidd", 0.01);
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
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if(mt1.tagCount != 0 && mt1 != null){
      m_robotContainer.drivetrain.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5,0.5,0.5));
      m_robotContainer.drivetrain.m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
    }
    m_field.setRobotPose(m_robotContainer.drivetrain.getPose());

    SmartDashboard.putNumber("Flywheel RPM", m_robotContainer.superStructure.getRPM());
    SmartDashboard.putNumber("GyroHeading", m_robotContainer.drivetrain.getHeading());
    SmartDashboard.putNumber("GyroAngleZ", m_robotContainer.drivetrain.m_gyro.getAngle());
    SmartDashboard.putNumber("LX", m_robotContainer.controller0.getLeftX());
    SmartDashboard.putNumber("LY", m_robotContainer.controller0.getLeftY());
    SmartDashboard.putNumber("RX", m_robotContainer.controller0.getRightX());
    SmartDashboard.putBoolean("FieldRelative", m_robotContainer.drivetrain.isFieldRel);
    SmartDashboard.putNumber("Rotator Position", m_robotContainer.superStructure.getRotatorPos());
    SmartDashboard.putNumber("Rotator Target", m_robotContainer.superStructure.getRotatorTarget());
    SmartDashboard.putNumber("RY", m_robotContainer.controller1.getRightY());

    double omegaRps = Units.degreesToRotations(m_robotContainer.drivetrain.getTurnRate());
    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    if (llMeasurement != null & llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 0.2) {
      m_robotContainer.drivetrain.resetOdometry(llMeasurement.pose);
    }

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
