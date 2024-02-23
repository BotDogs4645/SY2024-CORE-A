package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

public class LauncherCommand extends CommandBase{

    private Launcher launcher;
    private Limelight limelight;
    private Indexer indexer;
    private int tagId;
    private double desiredVelocity;

    public LauncherCommand (Launcher launcher, Indexer indexer, int tagId, double desiredVelocity){
        this.launcher = launcher;
        this.indexer = indexer;
        this.tagId = tagId;
        this.desiredVelocity = desiredVelocity;
        addRequirements(launcher, indexer);
    }
    @Override
    public void initialize() {
        Commands.sequence(
            //Feed the note
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> indexer.startIndexer(0.5), indexer),
                Commands.run(() -> launcher.startLauncher(0.5), launcher)
            ),
            //Stop feeding and aim
            Commands.deadline(
                Commands.waitSeconds(0.5),//Test to find the amount of time for note to leave intake but not shoot out of launcher
                Commands.run(() -> indexer.stopIndexer(), indexer),
                Commands.run(() -> launcher.stopLauncher(), launcher),
                Commands.waitSeconds(1),
                Commands.run(() -> launcher.aimLauncher(), launcher)
            ),

            // Wait a bit
            Commands.waitSeconds(0.1),

            // Shoot notes
            Commands.deadline(
                Commands.waitSeconds(1),
                Commands.run(() -> launcher.startLauncher(limelight.getLaunchVelocity().getAsDouble()), launcher)
            ),

            // Stop launcher
            Commands.runOnce(() -> launcher.stopLauncher(), launcher)
        ).handleInterrupt(() -> {
            launcher.stopLauncher();
        }).schedule();
    }
}
