package frc.robot;

public final class Constants {
    public static final class Turret {
        public static final class Ports {
            // bully electrical to get these
            public static final int yawMotor = 0;
            public static final int pitchMotor = 0;
            public static final int flywheelMotorA = 0;
            public static final int flywheelMotorB = 0;
            public static final int yawLimitSwitch = 0;
            public static final int pitchLimitSwitch = 0;
        }

        public static final class TunedCoefficients {
            public static final class YawPID {
                public static final double p = 0.00;
                public static final double i = 0.00;
                public static final double d = 0.00;
                public static final double errorThreshold = 0.50;
                public static final double minSafeAngle = -180.0;
                public static final double maxSafeAngle = 180.0;
                public static final double seekAdjustment = 0.10;
            }

            public static final class PitchPID {
                public static final double p = 0.00;
                public static final double i = 0.00;
                public static final double d = 0.00;
                public static final double errorThreshold = 0.50;
            }

            public static final class FlywheelPID {
                public static final double p = 0.00;
                public static final double i = 0.00;
                public static final double d = 0.00;
                public static final double errorThreshold = 100.0;
            }
        }

        public static final class FieldInfo {
            public static final double g = 9.8;
            public static final double turretShotDeltaY = 5.0;
            public static final double alpha = 45 * Math.PI / 180;
        }
    }
}