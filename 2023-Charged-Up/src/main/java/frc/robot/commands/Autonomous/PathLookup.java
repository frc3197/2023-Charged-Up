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
                ret = new PathContainer("practice", getSpeeds(SPEEDS.ONE), 1.25, true, true);
                break;                        
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