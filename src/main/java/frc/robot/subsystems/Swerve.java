package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controls the Swerve subsystem.
 * 
 * Swerve drive can move in any direction using its four distinct modules.
 * This also enables it to move in any direction, regardless of the direction of
 * the frame of the robot.
 * 
 * The subsystem itself simply controls logic for its four constituent modules.
 */
public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  public Swerve() {
    gyro = new Pigeon2(Constants.Swerve.pigeonID, "*");
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

    var pathConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.PathPlanner.driveKP, Constants.PathPlanner.driveKI, Constants.PathPlanner.driveKD),
        new PIDConstants(Constants.PathPlanner.turnKP, Constants.PathPlanner.turnKI, Constants.PathPlanner.turnKD),
        Constants.Swerve.maxSpeed,
        Constants.Swerve.wheelBase,
        new ReplanningConfig());

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        () -> Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates()),
        speeds -> setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds), false),
        pathConfig,
        () -> DriverStation.getAlliance().filter(a -> a == DriverStation.Alliance.Red).isPresent(),
        this);

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  /**
   * Drives Swerve towards the given target. For details on how this is done,
   * see {@link #drive(Translation2d, double, boolean, boolean)}.
   */
  public void driveToTag(Pose3d target) {
    SwerveModuleState[] states =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(target.getX(), target.getY(), target.getRotation().getAngle()));


    setModuleStates(states, false);
  }

  /**
   * Converts the desired rotation and translation into a list of module states
   * to pipe into {@link #setModuleStates(SwerveModuleState[], boolean)}.
   * This will essentially calculate what the speed and rotation of each module
   * needs to be in order to move in the provided direction and send that
   * information to each module.
   * 
   * @param translation the 2d translation offset; used to calculate the
   *                    magnitude and direction of movement
   * @param rotation    the desired rotation of the robot itself
   * @param fieldRelative whether movement is relative to the front of the robot
   *                      or to the field itself
   * @param isOpenLoop if swerve is currently being controlled in a feedback
   *                   loop; if not, this will use PID for speed control
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] states =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

    setModuleStates(states, isOpenLoop);
  }

  /**
   * Sets each module to the provided state.
   * @param states the array of states (one per swerve module)
   * @param isOpenLoop if swerve is currently being controlled in a feedback
   *                   loop; if not, this will use PID for speed control
   */
  public void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(states[mod.moduleNumber], isOpenLoop);
    }
  }

  /**
   * @return the robot position as estimated by the Swerve odometry
   */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * @return an array of module positions, one for each module
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for(int i = 0; i < mSwerveMods.length; i++) {
      positions[i] = mSwerveMods[i].getPosition();
    }
    return positions;
  }

  /**
   * Overrides the Swerve odometry's estimated position with the provided pose
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw().getValue())
        : Rotation2d.fromDegrees(gyro.getYaw().getValue());
  }

  public Pigeon2 getGyro() {
    return gyro;
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
