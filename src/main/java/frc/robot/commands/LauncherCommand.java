package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Intake;

public class LauncherCommand extends CommandBase{

    private Launcher launcher;
    private int tagId;
    private double desiredVelocity;
    private Intake intake;
    private boolean activated;

    public LauncherCommand (Launcher launcher, Intake intake, int tagId, double desiredVelocity, boolean activated){
        this.launcher = launcher;
        this.intake = intake;
        this.tagId = tagId;
        addRequirements(launcher, intake);
    }
    @Override
    public void initialize() {
        Commands.sequence(
            // Move launcher up
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> intake.setIntake(activated), intake),
                Commands.run(() -> launcher.startLauncher(desiredVelocity), launcher)
            ),

            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> launcher.aimLauncher(tagId), launcher)
            ),
            // Stop arm
            Commands.runOnce(() -> launcher.lockInAim(), launcher),

            // Wait a bit
            Commands.waitSeconds(0.1),

            // Shoot balls
            Commands.deadline(
                Commands.waitSeconds(1),
                Commands.run(() -> launcher.launchNote(), launcher)
            ),

            // Stop launcher
            Commands.runOnce(() -> launcher.stopLauncher(), launcher)
        ).handleInterrupt(() -> {
            launcher.lockInAim();
            launcher.stopLauncher();
        }).schedule();
    }
}
