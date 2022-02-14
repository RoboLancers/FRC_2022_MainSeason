package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.misc.LimeLight;
import frc.robot.subsystems.turret.commands.ActiveTrajectory;
import frc.robot.subsystems.turret.commands.MatchHeadingYaw;
import frc.robot.util.SparkMaxWrapper;

public class Turret extends SubsystemBase {
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
    }

    public boolean isReadyToShoot(){
        return (
            // Target must be in limelight view
            this.limelight.hasTarget() &&
            // Yaw degrees error must be within acceptable threshold
            Math.abs(this.limelight.yawOffset()) < Constants.Turret.TunedCoefficients.YawPID.errorThreshold &&
            // Pitch degrees error must be within acceptable threshold
            Math.abs(this.getPitch() - this.trajectory.theta) < Constants.Turret.TunedCoefficients.PitchPID.errorThreshold &&
            // Flywheel velocity error must be within acceptable threshold
            Math.abs(this.getFlywheelAverageSpeed() - this.trajectory.speed) < Constants.Turret.TunedCoefficients.FlywheelPID.errorThreshold
        );
    };
}
