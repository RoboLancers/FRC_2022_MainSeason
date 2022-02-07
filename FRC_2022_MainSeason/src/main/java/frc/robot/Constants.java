package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;

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

    public static final class Intake {
        public static final int ROLLER_PORT = 0;
        public static final int INDEXER_PORT = 0;
        public static final int RETRACTOR_CHANNEL_ONE = 0;
        public static final int RETRACTOR_CHANNEL_TWO = 0;
    }

    public static final class Lights {
        //this would be better suited for Variables.java
        public static final class ColorSensor {
        public static final Color kBlueTarget = Color.kBlue;
        public static final Color kRedTarget = Color.kRed;
        }
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