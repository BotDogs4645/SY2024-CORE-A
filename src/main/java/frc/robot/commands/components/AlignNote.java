package frc.robot.commands.components;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BackLimelight;
import frc.robot.subsystems.Swerve;
import java.util.Optional;
import java.util.OptionalDouble;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.CommandBuilder;

public class AlignNote extends Command {

    public final Swerve drivetrain;
    public final Field2d playingField;
    public final BackLimelight backLimelightInstance;

    public Optional<Command> advanceToTargetCommand = Optional.empty();

    public AlignNote(Swerve drivetrain, Field2d playingField, BackLimelight backLimelightInstance) {
        this.drivetrain = drivetrain;
        this.playingField = playingField;
        this.backLimelightInstance = backLimelightInstance;
    }

    @Override
    public void initialize() {
        OptionalDouble noteRotationalOffset = backLimelightInstance.determineTargetRotationalOffset();
        if (noteRotationalOffset.isPresent()) {
            advanceToTargetCommand = Optional.of(CommandBuilder.AdvanceToTarget(drivetrain, playingField, new Transform2d(playingField.getRobotPose().getTranslation(), new Rotation2d(noteRotationalOffset.getAsDouble(), 0))));
        }
    }

    @Override
    public void execute() {
        if (advanceToTargetCommand.isEmpty()) {
            OptionalDouble noteRotationalOffset = backLimelightInstance.determineTargetRotationalOffset();
            if (noteRotationalOffset.isPresent()) {
                advanceToTargetCommand = Optional.of(CommandBuilder.AdvanceToTarget(drivetrain, playingField, new Transform2d(playingField.getRobotPose().getTranslation(), new Rotation2d(noteRotationalOffset.getAsDouble(), 0))));
            }
        }
     }

     @Override
     public void end(boolean interrupted) {
        advanceToTargetCommand.get().cancel();
     }

     @Override
     public boolean isFinished() {
        return advanceToTargetCommand.isEmpty() || advanceToTargetCommand.get().isFinished();
     }
}