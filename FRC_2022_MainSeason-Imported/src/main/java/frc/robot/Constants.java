package frc.robot;

public final class Constants {
    public static final class Turret {
        public static final class Ports {
            public static final int yawMotor = 1;
            public static final int pitchMotor = 2;
            public static final int flywheelMotorA = 3;
            public static final int flywheelMotorB = 4;
        }

        public static final class TunedCoefficients {
            public static final class PitchPID {
                public static final double p = 0.00;
                public static final double i = 0.00;
                public static final double d = 0.00;
            }

            public static final class FlywheelPID {
                public static final double p = 0.00;
                public static final double i = 0.00;
                public static final double d = 0.00;
            }
        }
    }
}