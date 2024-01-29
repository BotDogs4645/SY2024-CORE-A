package frc.lib.util;

import frc.robot.Constants;

public class LaunchCalculations {
    private double verticalDistance, horizontalDistance;

    /**
     * Returns a LaunchCalculations object that can be used to calculate the
     * necessary launch velocity and angle for a note to reach a target, given the
     * vertical and horizontal distance to the target.
     *
     * @param verticalDistance  the vertical distance between the launcher and the target
     * @param horizontalDistance the forward distance between the launcher and the target
     */
    public LaunchCalculations(double verticalDistance, double horizontalDistance) {
        this.verticalDistance = verticalDistance;
        this.horizontalDistance = horizontalDistance;
    }

    /**
     * Calculates the vertical velocity vector component required to reach the target.
     * @return vertical velocity component in m/s
     */
    public double getVerticalVelocity() {
        return Math.sqrt(2 * Constants.Launcher.gravityAcceleration * verticalDistance);
    }

    /**
     * Calculates the horizontal velocity vector component required to reach the target.
     * @return horizontal velocity component in m/s
     */
    public double getHorizontalVelocity() {
        double timeToTravel = (-1 * getHorizontalVelocity() + Math.sqrt(
                Math.pow(getHorizontalVelocity(), 2) + 2 * Constants.Launcher.gravityAcceleration * verticalDistance))
                / (Constants.Launcher.gravityAcceleration);
        return horizontalDistance / timeToTravel;
    }

    /**
     * Calculates the launch velocity required to reach the target, in RPM, when launching at the calculated angle from getLaunchAngle().
     * @return launch velocity in RPM
     */
    public double getLaunchVelocity() {
        return toRPM(Math.sqrt(Math.pow(getVerticalVelocity(), 2) + Math.pow(getHorizontalVelocity(), 2)));
    }

    /**
     * Calculates the launch angle required to reach the target, in radians.
     * @return launch angle in radians
     */
    public double getLaunchAngle() {
        return Math.toDegrees(Math.atan(getVerticalVelocity() / getHorizontalVelocity()));
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
