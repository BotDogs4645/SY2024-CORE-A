package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Launcher;

public class LauncherCommand extends CommandBase{

    private Launcher launcher;
    private int tagId;

    public LauncherCommand (Launcher launcher, int tagId){
        this.launcher = launcher;
        this.tagId = tagId;
        addRequirements(launcher);
    }
    @Override
    public void initialize() {
        Commands.sequence(
            // Move arm up
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> launcher.aimLauncher(tagId), launcher)
            ),
            // Stop arm
            Commands.runOnce(() -> launcher.lockInAim(), launcher),

            // Wait a bit
            Commands.waitSeconds(0.1),

            // Shoot balls
            Commands.deadline(
                Commands.waitSeconds(1),
                Commands.run(() -> launcher.launchNote(), launcher)
            ),

            // Stop launcher
            Commands.runOnce(() -> launcher.stopLauncher(), launcher)
        ).handleInterrupt(() -> {
            launcher.lockInAim();
            launcher.stopLauncher();
        }).schedule();
    }
}