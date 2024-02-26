package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer;

public class IndexerCommand extends Command {

    private Indexer indexer;

    public IndexerCommand (Indexer indexer){
        this.indexer = indexer;
        addRequirements(indexer);
    }
    @Override
    public void initialize() {
        Commands.sequence(
            // Start up indexer
            Commands.runOnce(() -> indexer.startIndexer(0.5), indexer),
            // Wait a bit - note must leave intake
            Commands.waitSeconds(0.5)
        ).finallyDo(() -> indexer.stopIndexer()).schedule();
    }
}