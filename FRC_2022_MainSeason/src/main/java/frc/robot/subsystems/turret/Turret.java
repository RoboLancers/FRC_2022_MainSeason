package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.subsystems.TurretFlywheel;
import frc.robot.subsystems.turret.subsystems.TurretPitch;
import frc.robot.subsystems.turret.subsystems.yaw.TurretYaw;

public class Turret extends SubsystemBase {
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
    }

    public void adjust(){
        this.pitch.setPositionSetpoint(this.launchTrajectory.theta);
        this.flywheel.setVelocitySetpoint(this.launchTrajectory.speed);
    }

    public boolean isReadyToShoot(){
        return (
            this.yaw.isAligned() &&
            this.pitch.isAligned(this.launchTrajectory.theta) &&
            this.flywheel.isUpToSpeed(this.launchTrajectory.speed)
        );
    };
}
