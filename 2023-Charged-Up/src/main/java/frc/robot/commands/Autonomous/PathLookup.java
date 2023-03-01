// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

/** Add your docs here. */
public class PathLookup {

    private static double defaultTimeout = Double.POSITIVE_INFINITY;

    public static PathContainer getContainer(String pathName) {
        PathContainer ret = null;
        // PATH CONTAINER SYNTAX
        // new PathContainer(STRING PATH, DOUBLE ARRAY {MAX SPEED, MAX ACCELERATION},
        // DOUBLE TIMEOUT, BOOLEAN FIRST_PATH_IN AUTO)
        switch (pathName) {
            case "practice":
                ret = new PathContainer("practice", getSpeeds(SPEEDS.ONE), 1.25, false, true);
                break;
            case "practice2":
                ret = new PathContainer("practice2", getSpeeds(SPEEDS.ONE), 1.25, false, true);
                break;
            case "1.1B":
                ret = new PathContainer("1.1", getSpeeds(SPEEDS.TWO), 3, true, true);
                break;
            case "1.2B":
                ret = new PathContainer("1.2", getSpeeds(SPEEDS.TWO), 1, false, true);
                break;
            case "1.3.PLACEB":
                ret = new PathContainer("1.3.PLACE", getSpeeds(SPEEDS.ONE), 3, false, true);
                break;
            case "1.pre":
                ret = new PathContainer("1.pre", getSpeeds(SPEEDS.ONE), 2, false, false);
                break;
            case "3.1":
                ret = new PathContainer("3.1", new double[]{0.5, 0.5}, 4, true, false);
            case "leaveShort":
                ret = new PathContainer("leaveShort", new double[]{0.5, 0.5}, 5, false, false);
            
        }
        return ret;
    }

    private enum SPEEDS {
        ONE, TWO, THREE, FOUR, FIVE
    }

    private static double[] getSpeeds(SPEEDS speeds) {
        double[] ret;
        switch (speeds) {
            case ONE:
                ret = new double[] { 1, 1 };
                break;
            case TWO:
                ret = new double[] { 2, 2 };
                break;
            case THREE:
                ret = new double[] { 3, 3 };
                break;
            case FOUR:
                ret = new double[] { 4, 4 };
                break;
            default:
            case FIVE:
                ret = new double[] { 5, 5 };
                break;
        }
        return ret;
    }
}