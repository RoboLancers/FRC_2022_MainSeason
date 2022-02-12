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

<<<<<<< HEAD
    public double getYaw(){
        // TODO: NS
        
        // silence errors until this is implemented
        return 0.0;
    }

    public void setYawPower(double power){
        // TODO: NS
    };

    public double getPitch(){
        // TODO: NS

        // silence errors until this is implemented
        return 0.0;
    }

    public void setPitchPower(double power){
        // TODO: NS
    };

    public double getFlywheelAverageSpeed(){
        // TODO: NS

        // silence errors until this is implemented
        return 0.0;
    }

    public void setFlywheelPower(double power){
        // Very naive implementation, may need some more logic since it uses 2 independent motors
        this.flywheelMotorA.setPower(power);
        this.flywheelMotorB.setPower(power);
    };

    // Determine whether the yaw heading and derivative are valid for shooting
    public boolean headingIsAligned(){
=======
    public void adjust(){
        this.pitch.setPositionSetpoint(this.launchTrajectory.theta);
        this.flywheel.setVelocitySetpoint(this.launchTrajectory.speed);
    }

    public boolean isReadyToShoot(){
>>>>>>> fad666a7499e81a261984e98e3d90c1e5d32903f
        return (
            this.yaw.isAligned() &&
            this.pitch.isAligned(this.launchTrajectory.theta) &&
            this.flywheel.isUpToSpeed(this.launchTrajectory.speed)
        );
    };
}
