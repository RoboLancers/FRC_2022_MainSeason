package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Abstraction of Smart Dashboard
// Uses function overloading to simplify methods
public class SmarterDashboard {
    public static void put(String key, boolean value){
        SmartDashboard.putBoolean(key, value);
    }

    public static void put(String key, double value){
        SmartDashboard.putNumber(key, value);
    }

    public static void put(String key, String value){
        SmartDashboard.putString(key, value);
    }
}
