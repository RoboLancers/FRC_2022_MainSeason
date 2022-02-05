package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
    public static final class Climber {
        public static final int CLIMBER_PORT = 0;
    }

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

    public static final class Intake {
        public static final int ROLLER_PORT = 0;
        public static final int INDEXER_PORT = 0;
        public static final int RETRACTOR_CHANNEL_ONE = 0;
        public static final int RETRACTOR_CHANNEL_TWO = 0;
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
}