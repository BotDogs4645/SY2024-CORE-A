package frc.lib.util;

import frc.robot.Constants;

public class LaunchCalculations {
    private double verticalDistance, horizontalDistance;

    public LaunchCalculations(double verticalDistance, double horizontalDistance) {
        this.verticalDistance = verticalDistance;
        this.horizontalDistance = horizontalDistance;
    }
    
    public double getVerticalVelocity() {
        return Math.sqrt(2 * Constants.Launcher.gravityAcceleration * verticalDistance);
    }

    public double getHorizontalVelocity() {
        double timeToTravel = (-1 * getHorizontalVelocity() + Math.sqrt(Math.pow(getHorizontalVelocity(), 2) + 2 * Constants.Launcher.gravityAcceleration * verticalDistance))/(Constants.Launcher.gravityAcceleration);
        return horizontalDistance/timeToTravel;
    }

    public double getLaunchVelocity() {
        return Math.sqrt(Math.pow(getVerticalVelocity(), 2) + Math.pow(getHorizontalVelocity(), 2));
    }

    public double getLaunchAngle() {
        return Math.atan(getVerticalVelocity()/getHorizontalVelocity());
    }
}
