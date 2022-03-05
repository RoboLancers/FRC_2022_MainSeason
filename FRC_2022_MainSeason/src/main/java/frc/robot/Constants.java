package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;


public final class Constants {
    public static final class Climber {
        public static final int CLIMBER_PORT = 0;
        public static final double kSpoolRadius = 1.375;
        public static final double kRotationToInch = Math.PI*kSpoolRadius/15;
        public static final double kLowClimb = 48.75;
        public static final double kMidClimb = 60.25;
        public static final double kPower = 1;
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
                public static final double kP = 0.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final double kFF = 0.0;
                public static final double kMaxAbsoluteOutput = 0.0;
                // logic
                public static final double kErrorThreshold = 0.5;
                public static final double kMinSafeAngle = -180;
                public static final double kMaxSafeAngle = 180;
                public static final double kSeekAdjustment = 0.1;
            }

            public static final class PitchPID {
                // PID
                public static final double kP = 0.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final double kFF = 0.0;
                public static final double kMaxAbsoluteOutput = 0.0;
                // logic
                public static final double kErrorThreshold = 0.5;
            }

            public static final class FlywheelPID {
                // PID
                public static final double kP = 0.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final double kFF = 0.0;
                public static final double kMaxAbsoluteOutput = 0.0;
                // logic
                public static final double kErrorThreshold = 100;
                public static final double kMaxVelocity = 40;
                public static final double kCurrentSpikeThreshold = 0.0;
                public static final double kPostSpikeDelay = 0.2;
            }
        }

        public static final class PhysicsInfo {
            public static final double kGravity = 9.8;
            public static final double kTurretShotDeltaY = 5.0;
            public static final double kAlpha = 45 * Math.PI / 180;
            public static final double kSinAlpha = Math.sin(Constants.Turret.PhysicsInfo.kAlpha);
            public static final double kCosAlpha = Math.cos(Constants.Turret.PhysicsInfo.kAlpha);
            public static final double kTanAlpha = Math.tan(Constants.Turret.PhysicsInfo.kAlpha);
            public static final double kPitchMountAngle = 55;
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
        public static final int kProximityLimit = 1600;
        public static final int kIndexerPort = 13;
        public static final double kIndexerSpeed = 0.2;
        public static final int ktopSwitch = 1;
        public static final int kbottomSwitch = 0;
        public static final double kStandardIndexerSpeed = 0.1;
        public static final double kShootTime = 0;
        public static final double kIndexerOff = 0;
        public static final Color kRedTarget = new Color(1, 1, 1);
        public static final Color kBlueTarget = new Color(1, 1, 1);
    }

    public static final class AddressableLEDs {
        public static final int BlinkinPort = 0;
        public static final double YELLOW = 0.69;
        public static final double BLUE = 0.87;
        public static final double RED = 0.61;
        public static final double GREEN = 0.75;
        public static final double WHITE = 0.93;
        public static final double CRIMSON_GOLD_STROBE = 0.41;
        public static final double CRIMSON_GOLD_STROBE2 = 0.42;
        public static final double CRIMSON_GOLD_SOLID = 0.49;
        public static final double RAINBOW = -0.99;

    }

    public static final class Trajectory {
        public static final double ksVolts = 0.131;
        public static final double ksVoltSecondsPerMeter =  4.03;
        public static final double kaVoltSecondsSquaredPerMeter = 0.521;

        public static final double kTrackWidthMeters = 0.702;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kP = 0;

        public static final double kDistPerRot = (3.072/100);
        
    }
    
    public static final double kThrottleFilter = 1.25;
    public static final double kTurnFilter = 2;

}