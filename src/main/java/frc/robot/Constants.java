package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

/**
 * The Constants class contains constant values. Generally, when there are magic
 * numbers within your code (numbers that are not derived from a formula in the
 * code, like calibration values) they should go here. This encourages
 * cleanliness.
 */
public final class Constants {

  public static final class PathPlanner {
    public static final double driveKP = 2.5;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double turnKP = 0.3;
    public static final double turnKI = 0.0;
    public static final double turnKD = 0.75;
  }

  public static final class Vision {
    //all units in meters
    // sign of offsets follows std convention rel to swerve
    public static final double LimelightAngleDegrees = 30;
    public static final double LimelightOffsetZ = 10.5 * 0.0254;
    public static final double LimelightOffsetX = 0.32;
    public static final double LimelightOffsetY = 0.0;

  }
  public static class Launcher {
    public static final int angleMotorID = 18;
    public static final int topMotorID = 13;
    public static final int bottomMotorID = 14;

    public static final double gravityAcceleration = 9.81;
    public static final double launcherHeight = 0.574;
    public static final double launcherWheelRadius = 0.04826;

    public static final double ampHeight = 0.66;
    public static final double speakerHeight = 1.984;
    public static final double trapHeight = 1.436;
  }

  public static class Intake {
    public static final double intakeSpeed = 0.5;
  }

  public static final class Pneumatics {
    public static final int pcmCanID = 15;
    public static final int climberForward = 6;
    public static final int climberReverse = 7;
    public static final int ampGuideForward = 4;
    public static final int ampGuideReverse = 5;
  }

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 50;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(27.75);
    public static final double wheelBase = Units.inchesToMeters(34);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Trajectory Controller PID Values */
    public static final double xKP = 0.0;
    public static final double yKp = 0.0;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int canCoderID = 4;
      public static final int angleMotorID = 8;
      public static final int driveMotorID = 9;

      public static final boolean driveIsInverted = false;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(218.76953125);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveIsInverted);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int canCoderID = 5;
      public static final int angleMotorID = 16;
      public static final int driveMotorID = 17;
      
      public static final boolean driveIsInverted = true;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(12.744140625);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveIsInverted);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int canCoderID = 3;
      public static final int angleMotorID = 11;
      public static final int driveMotorID = 10;
      
      public static final boolean driveIsInverted = false;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(80.419921875 + 180);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveIsInverted);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int canCoderID = 2;
      public static final int angleMotorID = 6;
      public static final int driveMotorID = 7;
      
      public static final boolean driveIsInverted = false;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(120.673828125);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, driveIsInverted);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final int pdhID = 1;

  public static final int kDriverControllerPort = 0;

  public static class Limelight {

    /**
     * A key-value map, mapping each AprilTag to its position on the 2024
     * Crescendo board.
     * 
     * To retrieve a value, you can do {@code APRILTAGS.get(number)} and it will
     * return the transform of the april tag, or null if the number does not
     * have an associated AprilTag.
     * 
     * For a map of the game board and AprilTag positions, see
     * {@link https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf},
     * page 4.
     */
    public static final Map<Integer, Transform3d> APRILTAGS = Map.ofEntries(
        tag(-1, 0, 0, 0, 0),
        tag(1, 593.68, 9.68, 53.38, 120),
        tag(2, 637.21, 34.79, 53.38, 120),
        tag(3, 652.73, 196.17, 57.13, 180),
        tag(4, 652.73, 218.42, 57.13, 180),
        tag(5, 578.77, 323.00, 53.38, 270),
        tag(6, 72.5, 323.00, 53.38, 270),
        tag(7, -1.50, 218.42, 57.13, 0),
        tag(8, -1.50, 196.17, 57.13, 0),
        tag(9, 14.02, 34.79, 53.38, 60),
        tag(10, 57.54, 9.68, 53.38, 60),
        tag(11, 468.69, 146.19, 52.00, 300),
        tag(12, 468.69, 177.10, 52.00, 60),
        tag(13, 441.74, 161.62, 52.00, 180),
        tag(14, 209.48, 161.62, 52.00, 0),
        tag(15, 182.73, 177.10, 52.00, 120),
        tag(16, 182.73, 146.19, 52.00, 240));

    private static Map.Entry<Integer, Transform3d> tag(int id, double x, double y, double z, double rot) {
      final double inchesPerMeter = 39.37;

      return Map.entry(id, new Transform3d(
          new Translation3d(x / inchesPerMeter, y / inchesPerMeter, z / inchesPerMeter),
          new Rotation3d(rot, 0, 0)));
    }
  }

}
