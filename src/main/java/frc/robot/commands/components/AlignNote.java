package frc.robot.commands.components;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BackLimelight;
import frc.robot.subsystems.Swerve;
import java.util.Optional;
import frc.robot.Constants;
import frc.robot.commands.CommandBuilder;
import edu.wpi.first.math.geometry.Pose2d;

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
        Optional<Pose2d> relativeTargetPose = backLimelightInstance.getEstimatedTargetPose();

        // if (relativeTargetPose.isPresent() && Math.hypot(relativeTargetPose.get().getX(), relativeTargetPose.get().getY()) < Constants.Vision.BackLimelight.maximumAlignmentDistance) {
        //     advanceToTargetCommand = Optional.of(
        //         CommandBuilder.AdvanceToTarget(drivetrain, playingField, new Pose2d(new Translation2d(), relativeTargetPose.get().getRotation())).andThen(
        //             CommandBuilder.AdvanceToTarget(drivetrain, playingField, new Pose2d(relativeTargetPose.get().getTranslation(), new Rotation2d()))
        //         )
        //     );
        // }

        if (relativeTargetPose.isPresent() && Math.hypot(relativeTargetPose.get().getX(), relativeTargetPose.get().getY()) < Constants.Vision.BackLimelight.maximumAlignmentDistance) {
            advanceToTargetCommand = Optional.of(
                CommandBuilder.AdvanceToTarget(drivetrain, playingField, relativeTargetPose.get())
            );
        }
    }

    @Override
    public void execute() {
        if (advanceToTargetCommand.isEmpty()) {
            Optional<Pose2d> relativeTargetPose = backLimelightInstance.getEstimatedTargetPose();

            // if (relativeTargetPose.isPresent() && Math.hypot(relativeTargetPose.get().getX(), relativeTargetPose.get().getY()) < Constants.Vision.BackLimelight.maximumAlignmentDistance) {
                
            //     advanceToTargetCommand = Optional.of(
            //         CommandBuilder.AdvanceToTarget(drivetrain, playingField, new Pose2d(new Translation2d(), relativeTargetPose.get().getRotation())).andThen(
            //             CommandBuilder.AdvanceToTarget(drivetrain, playingField, new Pose2d(relativeTargetPose.get().getTranslation(), new Rotation2d()))
            //         )
            //     );
            // }

            if (relativeTargetPose.isPresent() && Math.hypot(relativeTargetPose.get().getX(), relativeTargetPose.get().getY()) < Constants.Vision.BackLimelight.maximumAlignmentDistance) {
                advanceToTargetCommand = Optional.of(
                    CommandBuilder.AdvanceToTarget(drivetrain, playingField, relativeTargetPose.get())
                );
            }
        }
     }

     @Override
     public void end(boolean interrupted) {
        if (advanceToTargetCommand.isPresent()) {
            advanceToTargetCommand.get().cancel();
        }
     }

     @Override
     public boolean isFinished() {
        if (advanceToTargetCommand.isPresent()) {
            return advanceToTargetCommand.get().isFinished();
        } else {
            return false;
        }
     }
}