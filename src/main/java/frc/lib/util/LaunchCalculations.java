package frc.lib.util;

import frc.robot.Constants;
import java.util.Optional;


public class LaunchCalculations {
    private AprilTag aprilTagInstance;

    /**
     * Returns a LaunchCalculations object that can be used to calculate the
     * necessary launch velocity and angle for a note to reach a target, given the
     * vertical and horizontal distance to the target.
     *
     * @param verticalDistance - the vertical distance between the launcher and the target
     * @param horizontalDistance - the forward distance between the launcher and the target
     */
    public LaunchCalculations(AprilTag aprilTagInstance) {
        this.aprilTagInstance = aprilTagInstance;
    }

    public Optional<Double> getVerticalDistance() {
        return Optional.of(aprilTagInstance.targetPos().get().getZ());
    }

    public Optional<Double> getHorizontalDistance() {
        return aprilTagInstance.getDirectDistance(Optional.of(aprilTagInstance.determinePosition().get().plus(aprilTagInstance.targetPos().get())));
    }

    /**
     * Calculates the vertical velocity vector component 
     * required to reach the target.
     * 
     * @return vertical velocity component in m/s
     */
    public Optional<Double> getVerticalVelocity() {
        Optional<Double> verticalDistance = getVerticalDistance();
        
        if (verticalDistance.isPresent()) {
            return Optional.of(Math.sqrt(2 * Constants.Launcher.gravityAcceleration * verticalDistance.get()));
        } else {
            return Optional.empty();
        }
    }

    /**
     * Calculates the horizontal velocity vector component 
     * required to reach the target.
     * 
     * @return horizontal velocity component in m/s
     */
    public Optional<Double> getHorizontalVelocity() {
        Optional<Double> verticalDistance = getVerticalDistance();
        Optional<Double> horizontalDistance = getHorizontalDistance();

        if (verticalDistance.isPresent() && horizontalDistance.isPresent()) {
            double timeToTravel = (-1 * getHorizontalVelocity().get() + Math.sqrt(
                Math.pow(getHorizontalVelocity().get(), 2) + 2 * Constants.Launcher.gravityAcceleration * verticalDistance.get()))
                / (Constants.Launcher.gravityAcceleration);

            return Optional.of(horizontalDistance.get() / timeToTravel);
        } else {
            return Optional.empty();
        }
    }

    /**
     * Calculates the launch velocity required to reach the target, in RPM, 
     * when launching at the calculated angle from getLaunchAngle().
     * 
     * @return launch velocity in RPM
     */
    public double getLaunchVelocity() {
        return toRPM(Math.sqrt(Math.pow(getVerticalVelocity().get(), 2) + Math.pow(getHorizontalVelocity().get(), 2)));
    }

    /**
     * Calculates the launch angle required to reach the target, in radians.
     * 
     * @return launch angle in radians
     */
    public double getLaunchAngle() {
        return Math.toDegrees(Math.atan(getVerticalVelocity().get() / getHorizontalVelocity().get()));
    }


    /**
     * Converts a velocity in m/s to RPM using the launcher wheel 
     * radius from Constants.Launcher.launcherWheelRadius.
     * 
     * @param velocity velocity in m/s
     * 
     * @return velocity in RPM
     */
    public double toRPM(double velocity) {
        return velocity * 60 / (2 * Math.PI * Constants.Launcher.launcherWheelRadius);
    }
}
