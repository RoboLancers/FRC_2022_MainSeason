package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.subsystems.TurretFlywheel;
import frc.robot.subsystems.turret.subsystems.TurretPitch;
import frc.robot.subsystems.turret.subsystems.TurretYaw;

public class Turret extends SubsystemBase {
    public boolean inHangMode = false;

    public TurretYaw yaw;
    public TurretPitch pitch;
    public TurretFlywheel flywheel;

    public LaunchTrajectory launchTrajectory;

    public Turret(Drivetrain driveTrain){
        this.yaw = new TurretYaw(this, driveTrain);
        this.pitch = new TurretPitch();
        this.flywheel = new TurretFlywheel();
    }

    public void adjust(){
        // this.pitch.positionSetpoint = this.launchTrajectory.theta;
        // this.flywheel.velocitySetpoint = this.launchTrajectory.speed;
    }

    public boolean inShootingRange()
    {
        return this.flywheel.getVelocity() < Constants.Turret.TunedCoefficients.FlywheelPID.kMaxVelocity;
    };

    public boolean isReadyToShoot(){
        return (
            this.yaw.isAligned() &&
            this.pitch.isAligned(this.launchTrajectory.theta) &&
            this.flywheel.isUpToSpeed(this.launchTrajectory.speed)
        );
    };
}
