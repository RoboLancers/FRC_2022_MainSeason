package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
    public static final class Climber {
        public static final int CLIMBER_PORT = 0;
    }

    public static final class Turret {
        public static final class Ports {
            // bully electrical to get these
            public static final int kYawMotor = 0;
            public static final int kPitchMotor = 0;
            public static final int kFlywheelMotorA = 0;
            public static final int kFlywheelMotorB = 0;
            public static final int kYawLimitSwitch = 0;
            public static final int kPitchLimitSwitch = 0;
        }

        public static final class TunedCoefficients {
            public static final class YawPID {
                // PID
                public static final double kP = 0.00;
                public static final double kI = 0.00;
                public static final double kD = 0.00;
                public static final double kFF = 0.00;
                public static final double kMaxAbsoluteOutput = 0.00;
                // logic
                public static final double kErrorThreshold = 0.50;
                public static final double kMinSafeAngle = -180.0;
                public static final double kMaxSafeAngle = 180.0;
                public static final double kSeekAdjustment = 0.10;
            }

            public static final class PitchPID {
                // PID
                public static final double kP = 0.00;
                public static final double kI = 0.00;
                public static final double kD = 0.00;
                public static final double kFF = 0.00;
                public static final double kMaxAbsoluteOutput = 0.00;
                // logic
                public static final double kErrorThreshold = 0.50;
            }

            public static final class FlywheelPID {
                // PID
                public static final double kP = 0.00;
                public static final double kI = 0.00;
                public static final double kD = 0.00;
                public static final double kFF = 0.00;
                public static final double kMaxAbsoluteOutput = 0.00;
                // logic
                public static final double kErrorThreshold = 100;
                public static final double kMaxVelocity = 40.0;
            }
        }

        public static final class PhysicsInfo {
            public static final double kGravity = 9.8;
            public static final double kTurretShotDeltaY = 5.0;
            public static final double kAlpha = 45 * Math.PI / 180;
            public static final double kSinAlpha = Math.sin(Constants.Turret.PhysicsInfo.kAlpha);
            public static final double kCosAlpha = Math.cos(Constants.Turret.PhysicsInfo.kAlpha);
            public static final double kTanAlpha = Math.tan(Constants.Turret.PhysicsInfo.kAlpha);
            public static final double kPitchMountAngle = 55.0;
        }
    }

    public static final class Intake {
        public static final int kRollerPort = 0;
        public static final int kRetractorChannelOne = 0;
        public static final int kRetractorChannelTwo = 0;
        public static final int kIndexerPort = 0;
        public static final int kPistonDeploy = 0;
        public static final int kPistonRetract = 0;
        public static final double kIntakePower = 0.6;
        public static final double kErrorMargin = 20;
        public static final double kIRollerOff = 0;
    }

    public static final class Indexer {
        public static final int kRedThreshold = 4000;
        public static final int kBlueThreshold = 4000;
        public static final int kProximityLimit = 1600;
        // Color sensor proximity?????
        public static final int kIndexerPort = 0;
        public static final double kIndexerSpeed = 0.2;
        public static final double kStandardIndexerSpeed = 0.1;
        public static final double kShootTime = 0;
        public static final double kIndexerOff = 0;
    }

    public static final class AddressableLEDs {
        public static final int BlinkinPort = 0;
        public static final double YELLOW = 0.69;
        public static final double BLUE = 0.87;
        public static final double RED = 0.61;
        public static final double GREEN = 0.75;
        public static final double RED_BLUE = 0;
        public static final double WHITE = 0;
        public static final double CRIMSON_GOLD = 0;
    }

    public static final class Trajectory {
        public static final double ksVolts = 0.131;
        public static final double ksVoltSecondsPerMeter =  4.03;
        public static final double kaVoltSecondsSquaredPerMeter = 0.521;

        public static final double kTrackWidthMeters = 0.702;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 19;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kP = 0;

        public static final double kDistPerRot = (3.072/100);
    }
}