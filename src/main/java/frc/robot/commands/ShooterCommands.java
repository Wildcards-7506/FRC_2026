// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCommands {
  private Shooter shooter;

  public ShooterCommands(Shooter shooter) {
    this.shooter = shooter;
  }

  public Command intakeCommand = new ParallelCommandGroup(
    // setIntakeCommand(1)
    // setLoaderCommand(1)
  );

  // public Command setLoaderCommand(double voltage) {
  //   return Commands        
  // }

  // public Command setFlywheelCommand (double voltage) {
    
  // }

  /**
   * 

    public Command setExtenderCommand(double setPoint) {
        return Commands.runOnce(() -> {
            extendTimer.reset();
            extendTimer.start();
        })
        .andThen(Commands.runOnce(() -> crane.setExtenderPosition(setPoint)))
        .until(() -> 
            (Math.abs(crane.getExtenderPosition() - setPoint) < CraneConstants.extendMargin  || 
            extendTimer.get() > 0.5 && Math.abs(crane.getElbowVelocity()) < 0.01)
        );
    }
   * @param voltage
   * @return
   */

  // public Command setIntakeCommand(double voltage) {
  //   return Commands.runOnce(() -> {
  //     shooter.toggleIntake(voltage);
  //   });
  // }
}
