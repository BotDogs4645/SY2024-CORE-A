package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeIndexer;

public class IntakeIndexerCommand extends Command {

    public Optional<IntakeIndexer> intakeIndexerInstance;
    public SequentialCommandGroup intakeIndexerCommand;

    public IntakeIndexerCommand(IntakeIndexer intakeIndexerInstance) {
        this.intakeIndexerInstance = Optional.of(intakeIndexerInstance);
    }

    @Override
    public void initialize() {
        intakeIndexerCommand = new SequentialCommandGroup (
            Commands.run(() -> {
                intakeIndexerInstance.get().runIntake(0.5);
                intakeIndexerInstance.get().runFeeder(0.5);
            }),
            
            Commands.waitUntil(() -> intakeIndexerInstance.get().hasNote()),

            Commands.runOnce(() -> {
                intakeIndexerInstance.get().stopIntake();
                intakeIndexerInstance.get().stopFeeder();
            })
        );
    }

    @Override
    public void end(boolean interrupted) {
        intakeIndexerCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return intakeIndexerInstance.isEmpty() || intakeIndexerCommand.isFinished();
    }
}
