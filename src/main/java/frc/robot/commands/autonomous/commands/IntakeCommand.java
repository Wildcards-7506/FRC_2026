package frc.robot.commands.autonomous.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    private final Timer timer = new Timer();
    private double duration;

    /**
     * Creates a new AutoSpinSucker command. Stops the sucker after a given duration.
     * 
     * @param duration Time in seconds to run the command.
     * @param volts Voltage to run the sucker.
     */
    public IntakeCommand(double duration) {
        this.duration = duration;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.m_robotContainer.superStructure.setIntakeVoltage(12);
        Robot.m_robotContainer.superStructure.setIntake2Voltage(12);
        Robot.m_robotContainer.superStructure.setLoaderVoltage(12);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.m_robotContainer.superStructure.setIntakeVoltage(0);
        Robot.m_robotContainer.superStructure.setIntake2Voltage(0);
        Robot.m_robotContainer.superStructure.setLoaderVoltage(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // Timeout in seconds
        // return timer.get() > duration || timer.get()>0.5 && Robot.crane.getSuckerCurrent() > 20;
        return timer.get() > duration;
    }
}
