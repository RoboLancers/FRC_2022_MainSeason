package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
    public static final class Climber {
        public static final int kClimberPort = 0;
    }

    public static final class Turret {
        public static final class Ports {
            // bully electrical to get these
            public static final int kYawPort = 0;
            public static final int kPitchPort = 0;
            public static final int kFlywheelPort1 = 0;
            public static final int kFlywheelPort2 = 0;
            public static final int kYawLimitPort = 0;
            public static final int kPitchLimitPort = 0;
        }

        public static final class TunedCoefficients {
            public static final class PitchPID {
                public static final double kP = 0.00;
                public static final double kI = 0.00;
                public static final double kD = 0.00;
            }

            public static final class FlywheelPID {
                public static final double kP = 0.00;
                public static final double kI = 0.00;
                public static final double kD = 0.00;
            }
        }
    }

    public static final class Intake {
        public static final int kIndexerPort = 0;
        public static final int kPistonDeploy = 0;
        public static final int kPistonRetract = 0;
        public static final int kRedThreshold = 4000;
        public static final int kBlueThreshold = 4000;
        public static final double kIntakePower = 200;
        // Speed of the intake
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
    }

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

    public static class Indexer {
        public static final int kProximityLimit = 1600;
        // Color sensor proximity?????
        public static final int kIndexerPort = 0;
        public static final double kIndexerSpeed = 0.2;
        public static final double kStandardIndexerSpeed = 0.1;
    }

    public static class LEDs {
        public static final int kLEDPort = 0;
    }
}