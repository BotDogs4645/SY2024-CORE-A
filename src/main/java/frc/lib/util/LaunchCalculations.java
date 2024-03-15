package frc.lib.util;

import frc.robot.Constants;

public class LaunchCalculations {
    
    /**
     * Calculates the vertical velocity vector component required to reach the target.
     * @return vertical velocity component in m/s
     */
    public double getVerticalVelocity(double verticalDistance) {
        return Math.sqrt(2 * Constants.Launcher.gravityAcceleration * verticalDistance);
    }

    /**
     * Calculates the horizontal velocity vector component required to reach the target.
     * @return horizontal velocity component in m/s
     */
    public double getHorizontalVelocity(double verticalDistance, double horizontalDistance) {
        double timeToTravel = (-1 * getHorizontalVelocity(verticalDistance, horizontalDistance) + Math.sqrt(
                Math.pow(getHorizontalVelocity(verticalDistance, horizontalDistance), 2) + 2 * Constants.Launcher.gravityAcceleration * verticalDistance))
                / (Constants.Launcher.gravityAcceleration);
        return horizontalDistance / timeToTravel;
    }

    /**
     * Calculates the launch velocity required to reach the target, in RPM, when launching at the calculated angle from getLaunchAngle().
     * @return launch velocity in RPM
     */
    public double getLaunchVelocity(double verticalDistance, double horizontalDistance) {
        return toRPM(Math.sqrt(Math.pow(getVerticalVelocity(verticalDistance), 2) + Math.pow(getHorizontalVelocity(verticalDistance, horizontalDistance), 2)));
    }

    /**
     * Calculates the launch angle required to reach the target, in radians.
     * @return launch angle in radians
     */
    public double getLaunchAngle(double verticalDistance, double horizontalDistance) {
        return Math.toDegrees(Math.atan(getVerticalVelocity(verticalDistance) / getHorizontalVelocity(verticalDistance, horizontalDistance)));
    }


    /**
     * Converts a velocity in m/s to RPM using the launcher wheel radius from Constants.Launcher.launcherWheelRadius.
     * @param velocity velocity in m/s
     * @return velocity in RPM
     */
    public double toRPM(double velocity) {
        return velocity * 60 / (2 * Math.PI * Constants.Launcher.launcherWheelRadius);
    }
}