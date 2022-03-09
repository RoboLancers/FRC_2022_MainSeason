package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;


public final class Constants {
    public static final class Drivetrain {
        public static final double kP = 0.0003;
        public static final double kI = 0.0;
        public static final double kD = 0.0005;
        public static final double kMaxAbsoluteError = 0.5;
    }

    public static final class Climber {
        public static final int CLIMBER_PORT = 0;
        public static final double kSpoolRadius = 1.375;
        public static final double kRotationToInch = Math.PI * kSpoolRadius/15;
        public static final double kLowClimb = 48.75;
        public static final double kMidClimb = 60.25;
        public static final double kPower = 1;
        public static final double kNegativePower = -1;
        public static final double kNormalHangCurrent = 1;
    }

    public static final class Turret {
        public static final class Ports {
            // bully electrical to get these
            public static final int kYawMotor = 13;
            public static final int kPitchMotor = 9;
            public static final int kFlywheelMotorA = 7;
            public static final int kFlywheelMotorB = 8;
            public static final int kYawLimitSwitch = 3;
            public static final int kPitchLimitSwitch = 2;
        }

        public static final class TunedCoefficients {
            public static final class YawPID {
                // PID
                public static final double kP = 0.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final double kFF = 0.0;
                public static final double kMaxAbsoluteOutput = 1.0;
                // logic
                public static final double kStoppedPosition = 1; // max absolute difference in degrees from 0 where the turret yaw considers itself to be at zero (for resetting turret before climbing)
                public static final double kErrorThreshold = 1; // max absolute error in degrees where the turret yaw considers itself aligned (for generalized release routine)
                // being extra careful about not overturning
                public static final double kMinSafeAngle = -170; // min turn angle in degrees for the turret yaw
                public static final double kMaxSafeAngle = 170; // max turn angle in degrees for the turret yaw
                public static final double kSeekAdjustment = 0.5; // magnitude of the change of the angle in the turret yaw in degrees (when looking for limelight)
            }

            public static final class PitchPID {
                public static final double kGearRatio = (12 * 464) / (18 * 360);
                // PID
                public static final double kP = 0.005;
                public static final double kI = 0.0;
                public static final double kD = 0.001;
                public static final double kFF = 0.028617;
                public static final double kMaxAbsoluteOutput = 1.0;
                // logic
                public static final double kStoppedPosition = 0.5; // max absolute difference in degrees from 0 where the turret pitch considers itself to be at zero (for resetting turret before climbing)
                public static final double kErrorThreshold = 0.5; // max absolute error in degrees where the turret pitch considers itself to be aligned (for generalized release routine)
            }

            public static final class FlywheelPID {
                public static final double kVelocityConversion = 4 * Math.PI;
                // PID
                public static final double kP = 0.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
                public static final double kFF = 0.00005;
                public static final double kMaxAbsoluteOutput = 1.0;
                // logic
                public static final double kStoppedVelocity = 100; // max absolute difference (in rpm?) from 0 where the turret flywheel considers itself to be at rest (for resetting turret before climbing)
                public static final double kErrorThreshold = 100; // max absolute error (in rpm?) where the turret flywheel considers itself to be up to speed (for generalized release routine)
                public static final double kMaxVelocity = 10.0; // maximum velocity the flywheel is capable of reaching (requires testing)
                public static final double kCurrentSpikeThreshold = 0.0; // ! - the threshold for if a current spike should trigger (for generalized release routine)
                public static final double kPostSpikeDelay = 0.1; // ! - the delay in seconds between a current spike and running progressBall (for generalized release routine)
            }
        }

        public static final class PhysicsInfo {
            // ! - requires testing
            public static final double minLimelightViewableDistance = 20;
            public static final double maxLimelightViewableDistance = 320.0;
            // Not necessary if we are using interpolation table
            public static final double kGravity = -386;
            public static final double kTurretShotDeltaY = 104 - 34;
            public static final double kUpperHubRadius = 24;
            public static final double kAlpha = 45 * Math.PI / 180;
            public static final double kSinAlpha = Math.sin(Constants.Turret.PhysicsInfo.kAlpha);
            public static final double kCosAlpha = Math.cos(Constants.Turret.PhysicsInfo.kAlpha);
            public static final double kTanAlpha = Math.tan(Constants.Turret.PhysicsInfo.kAlpha);
            public static final double kPitchMountAngle = 42;
        }
    }

    public static final class Intake {
        public static final int kRollerPort = 0;
        public static final int kRetractorChannelOne = 0;
        public static final int kRetractorChannelTwo = 0;
        public static final int kIndexerPort = 14;
        public static final int kPistonDeploy = 0;
        public static final int kPistonRetract = 0;
        public static final double kIntakePower = 0.6;
        public static final double kErrorMargin = 20;
        public static final double kIRollerOff = 0;
    }

    public static final class Indexer {
        public static final int kProximityLimit = 400;
        public static final int kIndexerPort = 14;
        public static final double kIndexerSpeed = 0.2;
        public static final int ktopSwitch = 1;
        public static final int kbottomSwitch = 0;
        public static final double kStandardIndexerSpeed = 0.1;
        public static final double kShootTime = 0;
        public static final double kIndexerOff = 0;
        public static final Color kRedTarget = new Color(1, 0, 0);
        public static final Color kBlueTarget = new Color(0, 0, 1);
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
    
    public static final double kThrottleFilter = 1.7;
    public static final double kTurnFilter = 1.5;
}