package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeIndexer;

public class IntakeIndexerCommand extends Command {

    public Optional<IntakeIndexer> intakeIndexerInstance;
    public SequentialCommandGroup intakeIndexerCommand;

    public IntakeIndexerCommand(IntakeIndexer intakeIndexerInstance) {
        this.intakeIndexerInstance = Optional.of(intakeIndexerInstance);

        addRequirements(intakeIndexerInstance);
    }

    @Override
    public void initialize() {
        intakeIndexerInstance.get().runIntake(0.5);
        intakeIndexerInstance.get().runFeeder(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        intakeIndexerInstance.get().stopIntake();
        intakeIndexerInstance.get().stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return intakeIndexerInstance.isEmpty() || intakeIndexerInstance.get().hasNote();
    }
}
