package frc.robot.commands.autonomous.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.SuperStructure;

public class GunCommand extends Command {
    private final Timer timer = new Timer();
//    private double duration;
    private static boolean stopGun;

    public GunCommand() {
//        this.duration = duration;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        stopGun = false;
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SuperStructure superStructure = Robot.m_robotContainer.superStructure;

//        superStructure.runIntake()
//                .alongWith(superStructure.rejectLoaderAuto())
//                .alongWith(superStructure.runIntake2()),
//                Commands.waitUntil(() -> Robot.flywheelHitTarget)
//                        .andThen(
//                                superStructure.runIntake()
//                                        .alongWith(superStructure.rejectLoaderAuto())
//                                        .alongWith(superStructure.runIntake2()))
//                        .alongWith(Agitator.runRight())
//                        .alongWith(Agitator.runLeft());
        superStructure.runIntake();
        superStructure.rejectLoaderAuto();
        superStructure.runIntake2();
        if (Robot.flywheelHitTarget) {
            Agitator.runRight();
            Agitator.runLeft();
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.m_robotContainer.superStructure.setIntakeVoltage(0);
        Robot.m_robotContainer.superStructure.setIntake2Voltage(0);
        Robot.m_robotContainer.superStructure.setLoaderVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return stopGun;
    }

    public static void doStopGun() {
        stopGun = true;
    }
}
