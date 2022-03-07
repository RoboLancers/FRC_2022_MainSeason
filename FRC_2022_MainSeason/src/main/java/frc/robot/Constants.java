package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;


public final class Constants {
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
                public static final double kStoppedPosition = 2 * Math.PI / 180; // max absolute difference in radians from 0 where the turret yaw considers itself to be at zero (for resetting turret before climbing)
                public static final double kErrorThreshold = 2 * Math.PI / 180; // max absolute error in radians where the turret yaw considers itself aligned (for generalized release routine)
                // being extra careful about not overturning
                public static final double kMinSafeAngle = -170 * Math.PI / 180; // min turn angle in radians for the turret yaw
                public static final double kMaxSafeAngle = 170 * Math.PI / 180; // max turn angle in radians for the turret yaw
                public static final double kSeekAdjustment = 0.5 * Math.PI / 180; // magnitude of the change of the angle in the turret yaw in radians  (when looking for limelight)
            }

            public static final class PitchPID {
                // 12m * 1 * 222 / 18
                public static final double ratio = (12 * 222) / (18 * 360);
                // PID
                public static double kP = 0.0;
                public static double kI = 0.0;
                public static double kD = 0.0;
                public static double kFF = 0.0;
                public static final double kMaxAbsoluteOutput = 1.0;
                // logic
                public static final double kStoppedPosition = 2 * Math.PI / 180; // max absolute difference in radians from 0 where the turret pitch considers itself to be at zero (for resetting turret before climbing)
                public static final double kErrorThreshold = 2 * Math.PI / 180; // max absolute error in radians where the turret pitch considers itself to be aligned (for generalized release routine)
            }

            public static final class FlywheelPID {
                // PID
                public static double kP = 0.0;
                public static double kI = 0.0;
                public static double kD = 0.0;
                public static double kFF = 0.12359;
                public static final double kMaxAbsoluteOutput = 1.0;
                // logic
                public static final double kStoppedVelocity = 0.25; // max absolute difference (in m/s?) from 0 where the turret flywheel considers itself to be at rest (for resetting turret before climbing)
                public static final double kErrorThreshold = 0.25; // max absolute error (in m/s?) where the turret flywheel considers itself to be up to speed (for generalized release routine)
                public static final double kMaxVelocity = 8.0; // maximum velocity the flywheel is capable of reaching
                public static final double kCurrentSpikeThreshold = 0.0; // ! - the threshold for if a current spike should trigger (for generalized release routine)
                public static final double kPostSpikeDelay = 0.1; // the delay in seconds between a current spike and running progressBall (for generalized release routine)
            }
        }

        public static final class PhysicsInfo {
            public static final double minLimelightViewableDistance = 0.0;
            public static final double maxLimelightViewableDistance = 0.0;
            // Not necessary if we are using interpolation table
            public static final double kGravity = -386;
            public static final double kTurretShotDeltaY = 104 - 34; // ! - subtract the height of the turret off the ground
            public static final double kUpperHubRadius = 24;
            public static final double kAlpha = 45 * Math.PI / 180;
            public static final double kSinAlpha = Math.sin(Constants.Turret.PhysicsInfo.kAlpha);
            public static final double kCosAlpha = Math.cos(Constants.Turret.PhysicsInfo.kAlpha);
            public static final double kTanAlpha = Math.tan(Constants.Turret.PhysicsInfo.kAlpha);
            public static final double kPitchMountAngle = 42 * Math.PI / 180; // ! check this with mechanical
        }
    }

    public static final class Intake {
        public static final int kRollerPort = 0;
        public static final int kRetractorChannelOne = 0;
        public static final int kRetractorChannelTwo = 0;
        public static final int kIndexerPort = 12;
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

    public static final class RGBConversion {
        public static final double a = 3.2404542;
        public static final double b = -1.5371385;
        public static final double c = -0.4985314;
        public static final double d = -0.9692660;
        public static final double e = 1.8760108;
        public static final double f = 0.0415560;
        public static final double g = 0.0556434;
        public static final double h = -0.2040259;
        public static final double i = 1.0572252;
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
    public static final double kTurnFilter = 3;

    public static class Drivetrain {
        public static final int kGyroPort = 1;
        public static final double kDistPerRot = (3.072/100);            
        public static final double kThrottleFilter = 1.5;
        public static final double kTurnFilter = 1.5;
        public static final double kMaxPower = 0.75;

        public static class LeftMotors {
            public static final int kLeftMotor1_Port = 0;
            public static final int kLeftMotor2_Port = 1;
            public static final int kLeftMotor3_Port = 2;
        }
        public static class RightMotors {
            public static final int kRightMotor1_Port = 3;
            public static final int kRightMotor2_Port = 4;
            public static final int kRightMotor3_Port = 5;
        }

    }
}