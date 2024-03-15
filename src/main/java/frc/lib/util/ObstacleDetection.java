package frc.lib.util;

import edu.wpi.first.math.geometry.Translation2d;

public class ObstacleDetection
{    
    /**
     * Determines whether the current path which the robot is on
     * will collide with the hitboxes of any of the obstacles 
     * specified in 'knownObstacles', as it is presented below.
     * 
     * @param currentPosition - The robot's current estimated position.
     * @param lineSlope - The slope of the robot's linear trajectory.
     * 
     * @return(s) whether the robot "should" 
     * continue along its current path.
     */
    public static boolean continueAlongPath(Translation2d currentPosition, double lineSlope) {

        for (double[] currentObstacle : knownObstacles) {
            double[] formattedLineData = formatLineData(currentPosition.getX(), currentPosition.getY(), lineSlope);

            if (!isTrajectoryValid(formattedLineData, currentObstacle)) {
                return false;
            }
        }

        return true;
    }

    /**
     * Determines whether a specified trajectory will 
     * intersect with a circle similarily provided in
     * the method's parameters.
     * 
     * @param lineData - The data pertaining to the line
     * which represents the robot's current trajectory.
     * @param circleCenterData - The data pertaining to 
     * the "target's" position, as well as radius.
     * 
     * @return(s) whether the given trajectory 
     * will intersect with the provided 
     * circle on a 2D plane.
     */
    public static boolean isTrajectoryValid(double[] lineData, double[] circleCenterData) {
        return Math.abs((lineData[0] * circleCenterData[0]) + (lineData[1] * circleCenterData[1]) + lineData[2]) / Math.sqrt(Math.pow(lineData[0], 2) + Math.pow(lineData[1], 2)) > circleCenterData[2];
    }

    /**
     * Formats the provided line data into a format which 
     * is able to be interpreted by the 'isTrajectoryValid'
     * method as it exists above (one which is known as 
     * "standard form").
     * 
     * @param xReference - The x-coordinate of
     * a specified point which exists on the 
     * line similarily specified in the method's
     * parameters. 
     * @param yReference - The y-coordinate of
     * a specified point which exists on the 
     * line similarily specified in the method's
     * parameters. 
     * @param lineSlope - The slope of the line
     * from which the data is interpreted.
     * 
     * @return(s) the formatted version 
     * of the provided line data.
     */
    public static double[] formatLineData(double xReference, double yReference, double lineSlope) {
        return new double[] {lineSlope, -1, (lineSlope * xReference) - yReference};
    }
    
    /** 
    * The obstacles shown below are formatted within a 
    * double array with the x-coordinate, y-coordinate, 
    * and aura radius in the zeroth, first, and second 
    * indicies of said storage respectively.
    */
    public static double[][] knownObstacles = {        
        {0, 0, 5},
    };
}