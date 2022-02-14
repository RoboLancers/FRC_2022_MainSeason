package frc.robot.util;

// TODO: refactor this into Utilities.java or vise versa

// Useful math utility functions
public class Maths {
    public static final double halfPI = 0.5 * Math.PI;
    private static final double radianCoefficient = Math.PI / 180;
    private static final double degreeCoefficient = 180 / Math.PI;

    // Clamp value, no sign/abs manipulation
    // min < max
    public static double naiveClamp(double number, double min, double max){
        return Math.min(Math.max(number, min), max);
    }

    // Clamp value, keep sign
    // min < max, min > 0
    public static double keepSignClamp(double number, double min, double max){
        return Math.signum(number) * naiveClamp(Math.abs(number), min, max);
    }

    // Square number, keep sign of number in result
    public static double squareKeepSign(double number){
        return Math.signum(number) * number * number;
    }

    // Take square root of absolute value of number, keep sign of number in result
    public static double rootKeepSign(double number){
        return Math.signum(number) * Math.sqrt(Math.abs(number));
    }

    // Convert degrees to radians
    public static double toRadians(double degrees){
        return degrees * Maths.radianCoefficient;
    }

    // Convert radians to degrees
    public static double toDegrees(double radians){
        return radians * Maths.degreeCoefficient;
    }

    public static double fastCosine(double x){
        return -0.4 * x * x + 0.95;
    }
}
