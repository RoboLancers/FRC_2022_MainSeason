package frc.robot;

public final class Constants {
    public static final class Turret {
        public static final class Ports {
            // bully electrical to get these
            public static final int kYawMotor = 0;
            public static final int kPitchMotor = 0;
            public static final int kFlywheelMotorA = 0;
            public static final int kFlywheelMotorB = 0;
            public static final int kKickWheel = 0;
            public static final int kYawLimitSwitch = 0;
            public static final int kPitchLimitSwitch = 0;
        }

        public static final class TunedCoefficients {
            public static final class YawPID {
                public static final double kP = 0.00;
                public static final double kI = 0.00;
                public static final double kD = 0.00;
                public static final double kErrorThreshold = 0.50;
                public static final double kMinSafeAngle = -180.0;
                public static final double kMaxSafeAngle = 180.0;
                public static final double kSeekAdjustment = 0.10;
            }

            public static final class PitchPID {
                public static final double kP = 0.00;
                public static final double kI = 0.00;
                public static final double kD = 0.00;
                public static final double kErrorThreshold = 0.50;
            }

            public static final class FlywheelPID {
                public static final double kP = 0.00;
                public static final double kI = 0.00;
                public static final double kD = 0.00;
                public static final double kErrorThreshold = 100.0;
            }
        }

        public static final class PhysicsInfo {
            public static final double kDeltaYawRecalculationThreshold = 2.0;
            public static final double kGravity = 9.8;
            public static final double kTurretShotDeltaY = 5.0;
            public static final double kAlpha = 45 * Math.PI / 180;
            public static final double kPitchMountAngle = 55.0;
        }
    }
}