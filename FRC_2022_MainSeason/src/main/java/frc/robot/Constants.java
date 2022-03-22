package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;


public final class Constants {

    public static final class Climber {
        public static final int CLIMBER_PORT1 = 14;
        public static final int CLIMBER_PORT2 = 15;
        public static final double kSpoolRadius = 1.375;
        public static final double kRotationToInch = Math.PI * kSpoolRadius/15;
        public static final double kLowClimb = 48.75;
        public static final double kMidClimb = 60.25;
        public static final double kPower = 1;
        public static final double kNegativePower = -1;
        public static final double kNormalHangCurrent = 1;
        public static final int kMaxHeight1 = 0;
        public static final int kMaxHeight2 = 0;
        public static final int kResetCurrent = 50;

    }

    public static final class Turret {
        public static final class Ports {
            public static final int kYawMotor = 13;
            public static final int kPitchMotor = 9;
            public static final int kFlywheelMotorA = 7;
            public static final int kFlywheelMotorB = 8;
            public static final int kYawLimitSwitch = 3;
            public static final int kPitchLimitSwitch = 2;
        }

        public static final class Yaw {
            // Used in drivetrain, not actual turret yaw motor
            public static final double kP = 0.016;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double kErrorThreshold = 1.0;
        }

        public static final class Pitch {
            public static final double kP = 0.75;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kFF = 0.0;
            public static final double kMaxAbsoluteVoltage = 0.25;

            public static final double kGearRatio = (12 * 464) / (18 * 360);

            public static final double kErrorThreshold = 0.1;
            public static final double kZeroAdjustment = 0.1;
            public static final double kMinSafeAngle = -0.5;
            public static final double kMaxSafeAngle = 12.5;
        }

        public static final class Flywheel {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kS = 0.1801;
            public static final double kV = 0.12747;
            public static final double kA = 0.0038965;
            public static final double kMaxAbsoluteVoltage = 1.0;

            public static final double kErrorThreshold = 25;
        }

        public static final class Physics {
            public static final double kDeltaY = 70;
            public static final double kMountAngle = 21;
            public static final double kUpperHubRadius = 24;
        }
    }

    public static final class Intake {
        public static final int kRollerPort = 0;
        public static final int kRetractorChannelOne = 6;
        public static final int kRetractorChannelTwo = 7;
        public static final int kIndexerPort = 11;
        public static final int kPistonDeploy = 6;
        public static final int kPistonRetract = 7;
        public static final double kIntakePower = 0.6;
        public static final double kErrorMargin = 20;
        public static final double kIRollerOff = 0;
    }

    public static final class Indexer {
        public static final int kProximityLimit = 400;
        public static final int kIndexerPort = 12;
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

    public static final class Drivetrain {
        public static final int kGyroPort = 1;
        public static final double kDistPerRot = (3.072/100);            
        public static final double kThrottleFilter = 1.5;
        public static final double kTurnFilter = 1.5;
        public static final double kMaxPower = 0.75;
        public static final double kP = 0.0003;
        public static final double kI = 0.0;
        public static final double kD = 0.0005;
        public static final double kMaxAbsoluteError = 0.5;
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