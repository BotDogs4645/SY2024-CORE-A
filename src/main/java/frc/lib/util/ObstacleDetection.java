package frc.lib.util;

import edu.wpi.first.math.geometry.Translation3d;

public class ObstacleDetection
{    

    public static boolean continueAlongPath(Translation3d currentPosition, double lineSlope) {

        for (double[] currentObstacle : knownObstacles) {
            double[] formattedLineData = formatLineData(currentPosition.getX(), currentPosition.getY(), lineSlope);

            if (!isTrajectoryValid(formattedLineData, currentObstacle)) {
                return false;
            }
        }

        return true;
    }

    public static boolean isTrajectoryValid(double[] lineData, double[] circleCenterData) {
        return Math.abs((lineData[0] * circleCenterData[0]) + (lineData[1] * circleCenterData[1]) + lineData[2]) / Math.sqrt(Math.pow(lineData[0], 2) + Math.pow(lineData[1], 2)) > circleCenterData[2];
    }

    public static double[] formatLineData(double xReference, double yReference, double lineSlope) {
        return new double[] {lineSlope, -1, (lineSlope * xReference) - yReference};
        // y - y1 = m1(x - x1)
        // y - y1 = m1x - m1x1
        // + y1 - m1x + y1 - m1x
        // ---------------------
        // y - m1x = y1 - m1x1
        // * -1       * -1
        // -------------------
        // m1x - y = m1x1 - y1  // CP
        // - (m1x1 + y1) - (m1x1 + y1)
        // ---------------------------
        // m1x - y - (m1x1 + y1) = 0
    }
    
    public static double[][] knownObstacles = {
        // The obstacles shown below are formatted within a double array with the x-coordinate, y-coordinate, and aura radius in the zeroth, first, and second indicies of said storage respectively.
        
        {0, 0, 5},
    };
}