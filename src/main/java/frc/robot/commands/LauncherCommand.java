package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Launcher;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

public class LauncherCommand extends CommandBase{

    private Launcher launcher;
    private int tagId;

    public LauncherCommand (Launcher launcher, Indexer indexer, int tagId){
        this.launcher = launcher;
        this.indexer = indexer;
        this.tagId = tagId;
        addRequirements(launcher, indexer);
    }
    @Override
    public void initialize() {
        Commands.sequence(
            //Feed the note
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> indexer.startIndexer(), indexer),
                Commands.run(() -> launcher.startLauncher(Constants.Launcher.feedVelocity), launcher) //velocity is placeholder value (needs to be changed)
            ),
            //Stop feeding and aim
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> indexer.stopIndexer(), indexer),
                Commands.run(() -> launcher.stopLauncher(), launcher),
                Commands.waitSeconds(1),
                Commands.run(() -> launcher.aimLauncher(launcher.getLaunchCalculations(tagId)), launcher)
            ),
            // Stop arm
            Commands.runOnce(() -> launcher.lockInAim(), launcher),

            // Wait a bit
            Commands.waitSeconds(0.1),

            // Shoot notes
            Commands.deadline(
                Commands.waitSeconds(1),
                Commands.run(() -> launcher.startLauncher(launcher.getLaunchCalculations(tagId)), launcher)
            ),

            // Stop launcher
            Commands.runOnce(() -> launcher.stopLauncher(), launcher)
        ).handleInterrupt(() -> {
            launcher.lockInAim();
            launcher.stopLauncher();
        }).schedule();
    }
}