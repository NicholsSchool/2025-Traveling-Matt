package org.firstinspires.ftc.teamcode.util;

public class AngleMath {
    /**
     * Adds angles and sets the sum to the range [-180, 180)
     *
     * @param angle1 the first angle in degrees
     * @param angle2 the second angle in degrees
     * @return the sum in degrees
     */
    public static double addAnglesDegrees(double angle1, double angle2) {
        double sum = angle1 + angle2;

        while(sum >= 180.0)
            sum -= 360.0;
        while(sum < -180.0)
            sum += 360.0;

        return sum;
    }

    /**
     * Adds angles and sets the sum to the range [-pi, pi)
     *
     * @param angle1 the first angle in radians
     * @param angle2 the second angle in radians
     * @return the sum in degrees
     */
    public static double addAnglesRadians(double angle1, double angle2) {
        double sum = angle1 + angle2;

        while(sum >= Math.PI)
            sum -= 2 * Math.PI;
        while(sum < -Math.PI)
            sum += 2 * Math.PI;

        return sum;
    }

    public static boolean driveAngleCheck(Vector driveVector, Vector positionVector){
        return positionVector.x * driveVector.x + positionVector.y * driveVector.y > 0;
    }

}
