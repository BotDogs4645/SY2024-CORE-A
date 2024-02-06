package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer;

public class IndexerCommand extends CommandBase{

    private Indexer indexer;

    public IndexerCommand (Indexer indexer){
        this.indexer = indexer;
        addRequirements(indexer);
    }
    @Override
    public void initialize() {
        Commands.sequence(
            // Move arm up
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> indexer.startIndexer(0.5), indexer)
            ),
            // Wait a bit
            Commands.waitSeconds(0.1),
            // Stop indexer
            Commands.runOnce(() -> indexer.stopIndexer(), indexer)
            ).handleInterrupt(() -> {
            indexer.stopIndexer();
        }).schedule();
    }
}
