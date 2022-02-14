package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.subsystems.flywheel.TurretFlywheel;
import frc.robot.subsystems.turret.subsystems.pitch.TurretPitch;
import frc.robot.subsystems.turret.subsystems.yaw.TurretYaw;

public class Turret extends SubsystemBase {
<<<<<<< HEAD
    // Limelight mounted on the turret base
    public LimeLight limelight;

    // Motor used to control the yaw of the turret
    // Motor used to control the pitch of the turret
    // First motor used to control the speed of the flywheel
    // Second motor used to control the speed of the flywheel
    public SparkMaxWrapper yawMotor, pitchMotor, flywheelMotorA, flywheelMotorB;

    // Switches used to home motors and prevent overturning
    public DigitalInput yawLimitSwitch;
    public DigitalInput pitchLimitSwitch;

    // Trajectory calculated using estimated field positions
    public Trajectory trajectory;

    public Turret(
        LimeLight limelight
    ){
        this.limelight = limelight;
        this.yawMotor = new SparkMaxWrapper(Constants.Turret.Ports.yawMotor);
        this.pitchMotor = new SparkMaxWrapper(Constants.Turret.Ports.pitchMotor);
        this.flywheelMotorA = new SparkMaxWrapper(Constants.Turret.Ports.FLYWHEEL_PORT);
        this.flywheelMotorB = new SparkMaxWrapper(Constants.Turret.Ports.KICKWHEEL_PORT);
        this.yawLimitSwitch = new DigitalInput(Constants.Turret.Ports.yawLimitSwitch);
        this.pitchLimitSwitch = new DigitalInput(Constants.Turret.Ports.pitchLimitSwitch);

        new PerpetualCommand(new ActiveTrajectory(this));
        new PerpetualCommand(new MatchHeadingYaw(this));
    }
    
    public void adjust(){
        this.pitch.setPositionSetpoint(this.launchTrajectory.theta);
        this.flywheel.setVelocitySetpoint(this.launchTrajectory.speed);
=======
    public TurretYaw yaw;
    public TurretPitch pitch;
    public TurretFlywheel flywheel;

    public LaunchTrajectory launchTrajectory;

    public Turret(){
        this.yaw = new TurretYaw((LaunchTrajectory newLaunchTrajectory) -> {
            this.launchTrajectory = newLaunchTrajectory;
        });
        this.pitch = new TurretPitch();
        this.flywheel = new TurretFlywheel();
>>>>>>> 64592c4a8cccc6cf78f51395f575e44f97cfff84
    }

    public boolean isReadyToShoot(){
        return (
            this.yaw.isAligned() &&
            this.pitch.isAligned(this.launchTrajectory.theta) &&
            this.flywheel.isUpToSpeed(this.launchTrajectory.speed)
        );
    };
}
