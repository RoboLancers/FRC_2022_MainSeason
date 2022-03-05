package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.commands.TeleopDrive;
import frc.robot.util.XboxController;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase{
    private final CANSparkMax leftMotor1 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax leftMotor2 = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax leftMotor3 = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final MotorControllerGroup leftMotors = new MotorControllerGroup(
        leftMotor1, leftMotor2, leftMotor3
    );

    private final CANSparkMax rightMotor1 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor2 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor3 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final MotorControllerGroup rightMotors = new MotorControllerGroup(
        rightMotor1, rightMotor2, rightMotor3
    );

    private final DifferentialDrive difDrive = new DifferentialDrive(leftMotors, rightMotors);

    private final DifferentialDriveOdometry odometry;

    // reverse the encoders to match the reversed motors of the right side.
    private final RelativeEncoder rightEncoder = rightMotor1.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor1.getEncoder();

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);    

    private final Field2d m_field = new Field2d();

    private final SlewRateLimiter throttleFilter = new SlewRateLimiter(Constants.kThrottleFilter);
    private final SlewRateLimiter turnFilter = new SlewRateLimiter(Constants.kTurnFilter);
    
    public Drivetrain(XboxController driverController){
        // Reverses the right motors.
        rightMotor1.setInverted(true);
        rightMotor2.setInverted(true);
        rightMotor3.setInverted(true);

        leftMotor1.setInverted(false);
        leftMotor2.setInverted(false);
        leftMotor3.setInverted(false);

        leftMotor1.setIdleMode(IdleMode.kBrake);
        leftMotor2.setIdleMode(IdleMode.kBrake);
        leftMotor3.setIdleMode(IdleMode.kBrake);

        rightMotor1.setIdleMode(IdleMode.kBrake);
        rightMotor2.setIdleMode(IdleMode.kBrake);
        rightMotor3.setIdleMode(IdleMode.kBrake);

        // Sets the distance per pulse to the pre-defined constant we calculated for both encoders.
        rightEncoder.setPositionConversionFactor(Constants.Trajectory.kDistPerRot);
        leftEncoder.setPositionConversionFactor(Constants.Trajectory.kDistPerRot);

        resetEncoders();
        
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        //initDefaultCommand(driverController);
    }

    // private void initDefaultCommand(XboxController driverController){
    //     setDefaultCommand(new TeleopDrive(this, driverController));
    

    // Constantly updates the odometry of the robot with the rotation and the distance traveled.
    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        m_field.setRobotPose(odometry.getPoseMeters());
    }

    // Returns the pose of the robot.
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // Returns the current speed of the wheels of the robot.
    public DifferentialDriveWheelSpeeds getWheelSpeeds() { 
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    // Resets the odometry, both rotation and distance traveled.
    public void resetOdometry(Pose2d pose) {
        gyro.reset();
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    // Drives the robot with arcade controls.
    public void arcadeDrive(double throttle, double turn) {
        difDrive.arcadeDrive(throttleFilter.calculate(throttle), turn*0.6, false);
        // if (throttle == 0 && turn == 0) {
        //     tankDriveVolts(0, 0);
        // }
    }

    public void setDrivePower(double power) {
        leftMotors.set(power);
        rightMotors.set(power);
    }

    // Controls the left and right side motors directly with voltage.
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        difDrive.feed();
    }

    // Resets the record values of both sides of encoders.
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    // These methods are never used?
    // Returns the average of the distances between both sides of encoders.
    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    // Returns the left encoders.
    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    // Returns the right encoders.
    public RelativeEncoder getRightEncoder() {
        return rightEncoder;
    }

    // Sets the max output of the drive. Used for scaling the drive to drive more slowly.
    public void setMaxOutput(double maxOutPut) {
        difDrive.setMaxOutput(maxOutPut);
    }

    public double getLeftVoltage() {
        return leftMotors.get();
    }

    public double getRightVoltage() {
        return rightMotors.get();
    }

    // Sets the recorded heading to 0. Makes new direction the 0 heading.
    public void zeroHeading() {
        gyro.reset();
    }

    // Returns the direction the robot is facing in degrees from -180 to 180 degrees. 
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    // Returns the rate at which the robot is turning in degrees per second.
    public double getTurnRate() {
       return -gyro.getRate();
    }
}