package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase{
    //The motors on the left side of the drivetrain
    private final CANSparkMax leftMotor1 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final MotorControllerGroup leftMotors = new MotorControllerGroup(
            leftMotor1,
            new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless), 
            new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless));
    //The motors on the right side of the drivetrain
    private final CANSparkMax rightMotor1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(
            rightMotor1,
            new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless),
            new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless));
    //A differential drive object that takes in both motor sides. 
    private final DifferentialDrive difDrive = new DifferentialDrive(leftMotors, rightMotors);
    //An odometry object to keep track of robot pose.
    private final DifferentialDriveOdometry odometry;
    //The encoders on the right motors. Reverse the encoders to match the reversed motors of the right side.
    private final RelativeEncoder rightEncoder = rightMotor1.getEncoder();
    //The encoders on the left motors.
    private final RelativeEncoder leftEncoder = leftMotor1.getEncoder();
    //The PigeonIMU gyro.
    //private final WPI_PigeonIMU gyro; //Check port
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);    
    //Field2d object to track pose in Glass
    private final Field2d m_field = new Field2d();
    private final SlewRateLimiter throttleFilter = new SlewRateLimiter(Constants.kThrottleFilter);
    private final SlewRateLimiter turnFilter = new SlewRateLimiter(Constants.kTurnFilter);
    

    //Drivetrain
    public Drivetrain(){
        
        //Reverses the right motors.
        rightMotors.setInverted(true);
        //Sets the distance per pulse to the pre-defined constant we calculated for both encoders.
        rightEncoder.setPositionConversionFactor(Constants.Trajectory.kDistPerRot); // create EncoderDistancePerPulse constant later
        leftEncoder.setPositionConversionFactor(Constants.Trajectory.kDistPerRot); // same thing

        //Resets the encoders to 0.
        resetEncoders();
        
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    }

    @Override
    //Constantly updates the odometry of the robot with the rotation and the distance traveled.
    public void periodic() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        m_field.setRobotPose(odometry.getPoseMeters());
    }

    //Returns the pose of the robot.
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    //Returns the current speed of the wheels of the robot.
    public DifferentialDriveWheelSpeeds getWheelSpeeds() { 
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), -rightEncoder.getVelocity());
    }

    //Resets the odometry, both rotation and distance traveled.
    public void resetOdometry(Pose2d pose) {
        gyro.reset();
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }


    //Drives the robot with arcade controls.
    public void arcadeDrive(double throttle, double turn) {
        difDrive.curvatureDrive(throttleFilter.calculate(throttle), turnFilter.calculate(turn), throttle < 0.05);
        // if (throttle == 0 && turn == 0) {
        //     tankDriveVolts(0, 0);
        // }
        
    }

    //Controls the left and right side motors directly with voltage.
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        difDrive.feed();
    }

    //Resets the record values of both sides of encoders.
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }


    //These methods are never used?
    //Returns the average of the distances between both sides of encoders.
    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    //Returns the left encoders.
    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    //Returns the right encoders.
    public RelativeEncoder getRightEncoder() {
        return rightEncoder;
    }

    //Sets the max output of the drive. Used for scaling the drive to drive more slowly.
    public void setMaxOutput(double maxOutPut) {
        difDrive.setMaxOutput(maxOutPut);
    }

    public double getLeftVoltage() {
        return leftMotors.get();
    }

    public double getRightVoltage() {
        return rightMotors.get();
    }

    //Sets the recorded heading to 0. Makes new direction the 0 heading.
    public void zeroHeading() {
        gyro.reset();
    }

    //Returns the direction the robot is facing in degrees from -180 to 180 degrees. 
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    //Returns the rate at which the robot is turning in degrees per second.
    public double getTurnRate() {
       return -gyro.getRate();
    }


    
}





