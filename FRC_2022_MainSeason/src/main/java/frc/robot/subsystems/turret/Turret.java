package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.misc.LimeLight;
import frc.robot.subsystems.turret.subsystems.TurretFlywheel;
import frc.robot.subsystems.turret.subsystems.TurretPitch;
<<<<<<< HEAD
// import frc.robot.subsystems.turret.subsystems.yaw.TurretYaw;
=======
>>>>>>> 51c0f53f0f499ead1f5f5777d9d603d7599c6767

public class Turret extends SubsystemBase {
    public boolean inHangMode = false;

<<<<<<< HEAD
    // public TurretYaw yaw;
=======
    public LimeLight limelight;

>>>>>>> 51c0f53f0f499ead1f5f5777d9d603d7599c6767
    public TurretPitch pitch;
    public TurretFlywheel flywheel;

    public LaunchTrajectory launchTrajectory = new LaunchTrajectory(0, 0);

<<<<<<< HEAD
    public Turret(Drivetrain driveTrain){
        // this.yaw = new TurretYaw(this, driveTrain);
=======
    public Turret(){
        this.limelight = new LimeLight();
>>>>>>> 51c0f53f0f499ead1f5f5777d9d603d7599c6767
        this.pitch = new TurretPitch();
        this.flywheel = new TurretFlywheel();
    }

    public boolean inShootingRange(){
        return (
            this.limelight.hasTarget() &&
            LaunchTrajectory.estimateDistance(this.limelight.pitchOffset()) < Constants.Turret.Physics.kMaxShootDistance
        );
    }

    public boolean isReadyToShoot(){
        // TODO: make this take pitch and flywheel setpoints as args
        return (
<<<<<<< HEAD
            // this.yaw.isAligned() &&
            this.pitch.isAligned(this.launchTrajectory.theta) &&
            this.flywheel.isUpToSpeed(this.launchTrajectory.speed)
=======
            this.limelight.hasTarget() &&
            Math.abs(this.limelight.yawOffset()) < Constants.Turret.Yaw.kErrorThreshold &&

            this.pitch.isAligned(0) &&
            this.flywheel.isUpToSpeed(0)
>>>>>>> 51c0f53f0f499ead1f5f5777d9d603d7599c6767
        );
    };
}