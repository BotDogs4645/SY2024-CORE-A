package frc.lib.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public final class CTREConfigs {
  public CANcoderConfiguration swerveCanCoderConfig;
  private MagnetSensorConfigs swerveMagnetSensorConfig;

  public CTREConfigs() {
    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig = new CANcoderConfiguration();
    swerveMagnetSensorConfig = new MagnetSensorConfigs();
    swerveMagnetSensorConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    swerveMagnetSensorConfig.SensorDirection = Constants.Swerve.canCoderInvert ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
    swerveCanCoderConfig.MagnetSensor = swerveMagnetSensorConfig;
  }
}
