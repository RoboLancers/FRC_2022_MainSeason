package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.misc.LimeLight;
import frc.robot.subsystems.turret.subsystems.TurretFlywheel;
import frc.robot.subsystems.turret.subsystems.TurretPitch;

public class Turret extends SubsystemBase {
    public boolean inHangMode = false;

    public LimeLight limelight;

    public TurretPitch pitch;
    public TurretFlywheel flywheel;

    private LaunchTrajectory launchTrajectory;

    public Turret(){
        this.limelight = new LimeLight();
        this.pitch = new TurretPitch();
        this.flywheel = new TurretFlywheel();
    }

    @Override
    public void periodic(){
        if(!SmartDashboard.getBoolean("Manual Entry", true)){
            this.pitch.positionSetpoint = this.launchTrajectory.theta;
            this.flywheel.velocitySetpoint = this.launchTrajectory.speed;
        }
        SmartDashboard.putBoolean("Ready To Shoot", this.isReadyToShoot());
    }

    public void resetLaunchTrajectory(){
        this.launchTrajectory = new LaunchTrajectory(0.0, 0.0);
    }

    public void setLaunchTrajectory(LaunchTrajectory newLaunchTrajectory){
        this.launchTrajectory.theta = newLaunchTrajectory.theta;
        this.launchTrajectory.speed = newLaunchTrajectory.speed;
    }

    public boolean inShootingRange(){
        // TODO: measure actual max distance the flywheel can reach
        return true;
    }

    public boolean isReadyToShoot(){
        return (
            // TODO: make a constant for yaw error threshold
            this.limelight.hasTarget() &&
            Math.abs(this.limelight.yawOffset()) < 0.25 &&

            this.pitch.isAligned() &&
            this.flywheel.isUpToSpeed()
        );
    };
}