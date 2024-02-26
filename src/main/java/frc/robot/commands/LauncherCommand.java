package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Indexer;

public class LauncherCommand extends Command {

    private Launcher launcher;
    private Limelight limelight;
    private Indexer indexer;

    public LauncherCommand (Launcher launcher, Indexer indexer){
        this.launcher = launcher;
        this.indexer = indexer;
        addRequirements(launcher, indexer);
    }
    @Override
    public void initialize() {
        Commands.sequence(
            //Feed the note
            Commands.runOnce(() -> indexer.startIndexer(0.5), indexer),
            Commands.runOnce(() -> launcher.startLauncher(0.5), launcher),
            Commands.waitSeconds(0.5), //Test to find the amount of time for note to leave intake but not shoot out of launcher

            //Stop feeding and aim
            Commands.runOnce(() -> indexer.stopIndexer(), indexer),
            Commands.runOnce(() -> launcher.stopLauncher(), launcher),
            Commands.waitSeconds(0.5),

            // Aim launcher
            Commands.runOnce(() -> launcher.aimLauncher(), launcher),
            Commands.waitSeconds(1),

            // Shoot notes
            Commands.runOnce(() -> launcher.startLauncher(limelight.getLaunchVelocity().getAsDouble()), launcher),
            Commands.waitSeconds(1)
        ).finallyDo(() -> launcher.stopLauncher()).schedule();
    }
}
