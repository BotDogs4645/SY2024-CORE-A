package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeIndexer;

public class IntakeIndexerCommand extends Command {

    public IntakeIndexer intakeIndexerInstance;
    public SequentialCommandGroup intakeIndexerCommand;

    public IntakeIndexerCommand(IntakeIndexer intakeIndexerInstance) {
        this.intakeIndexerInstance = intakeIndexerInstance;

        addRequirements(intakeIndexerInstance);
    }

    @Override
    public void initialize() {
        intakeIndexerInstance.runIntakeIndexer();
    }

    @Override
    public void end(boolean interrupted) {
        intakeIndexerInstance.haltIntakeIndexer();
    }

    @Override
    public boolean isFinished() {
        return intakeIndexerInstance.hasNote();
    }
}
