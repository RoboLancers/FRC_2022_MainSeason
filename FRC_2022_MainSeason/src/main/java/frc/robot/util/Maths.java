package frc.robot.util;

// Useful math utility functions
public class Maths {
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
}
