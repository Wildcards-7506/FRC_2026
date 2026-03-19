package frc.robot.commands.autonomous.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.SuperStructureConstants;

public class RotatorDownCommand extends SequentialCommandGroup {
    public RotatorDownCommand(RobotContainer container) {
        System.out.println("Stow preset called");
        // addRequirements(Robot.crane);
        addRequirements(container.superStructure);
        addCommands(
            container.superStructure.setRotator(SuperStructureConstants.rotatorMax)
            // 2025 example:
            // Commands.runOnce(() -> Robot.led.enableStreamer = false),
            // //Simultaneously move elbow, extender, and wrist to the appropriate setpoints
            // new ParallelCommandGroup(
            //     new SetWristCommand(CraneConstants.kWristHardDeck),
            //     new SetExtenderCommand(CraneConstants.kExtenderLimit1),
            //     new SetElbowCommand(CraneConstants.kElbowHardDeck + 10)), // Pause 10 degrees above to extend before moving to stow
            // new SetExtenderCommand(CraneConstants.kExtenderStow),
            // new SetElbowCommand(CraneConstants.kElbowHardDeck),
            // //prevent the commands from being scheduled more than once
            // Commands.runOnce(() -> Robot.crane.runSetpoint = false),
            // Commands.runOnce(() -> Robot.led.streamerBrightness = 0),
            // Commands.runOnce(() -> Robot.led.enableStreamer = true)
        );
    }
}