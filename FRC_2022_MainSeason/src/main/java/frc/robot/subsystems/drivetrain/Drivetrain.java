package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase{
    //The motors on the left side of the drivetrain
    private final CANSparkMax leftMotor1 = new CANSparkMax(Constants.Drivetrain.LeftMotors.kLeftMotor1_Port, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final MotorControllerGroup leftMotors = new MotorControllerGroup(
            leftMotor1,
            new CANSparkMax(Constants.Drivetrain.LeftMotors.kLeftMotor2_Port, CANSparkMaxLowLevel.MotorType.kBrushless), 
            new CANSparkMax(Constants.Drivetrain.LeftMotors.kLeftMotor3_Port, CANSparkMaxLowLevel.MotorType.kBrushless));
    //The motors on the right side of the drivetrain
    private final CANSparkMax rightMotor1 = new CANSparkMax(Constants.Drivetrain.RightMotors.kRightMotor1_Port, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(
            rightMotor1,
            new CANSparkMax(Constants.Drivetrain.RightMotors.kRightMotor2_Port, CANSparkMaxLowLevel.MotorType.kBrushless),
            new CANSparkMax(Constants.Drivetrain.RightMotors.kRightMotor3_Port, CANSparkMaxLowLevel.MotorType.kBrushless));
    //A differential drive object that takes in both motor sides. 
    private final DifferentialDrive difDrive = new DifferentialDrive(leftMotors, rightMotors);
    //An odometry object to keep track of robot pose.
    private final DifferentialDriveOdometry odometry;
    //The encoders on the right motors. Reverse the encoders to match the reversed motors of the right side.
    private final RelativeEncoder rightEncoder = rightMotor1.getEncoder();
    //The encoders on the left motors.
    private final RelativeEncoder leftEncoder = leftMotor1.getEncoder();
    //The PigeonIMU gyro.
    private final WPI_PigeonIMU gyro; //Check port
    //Field2d object to track pose in Glass
    private final Field2d m_field;
  


    //Drivetrain
    public Drivetrain(Field2d field, WPI_PigeonIMU gyro){
        
        //Reverses the right motors.
        rightMotors.setInverted(true);
        //Sets the distance per rotation to the pre-defined constant we calculated for both encoders.
        rightEncoder.setPositionConversionFactor(frc.robot.Constants.Drivetrain.kDistPerRot);
        leftEncoder.setPositionConversionFactor(frc.robot.Constants.Drivetrain.kDistPerRot);
        this.m_field = field;
        this.gyro = gyro;
        //Resets the encoders to 0.
        resetEncoders();
        
        setMaxOutput(Constants.Drivetrain.kMaxPower);
        
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
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }


    //Drives the robot with arcade controls.
    public void arcadeDrive(double throttle, double turn) {
        difDrive.arcadeDrive(throttle, turn);
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
    public void setMaxOutput(double maxOutput) {
        difDrive.setMaxOutput(maxOutput);
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





