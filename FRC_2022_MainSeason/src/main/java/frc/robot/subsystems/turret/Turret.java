package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.misc.LimeLight;
import frc.robot.subsystems.turret.subsystems.TurretFlywheel;
import frc.robot.subsystems.turret.subsystems.TurretPitch;

public class Turret extends SubsystemBase {
    public boolean inHangMode = false;

    public LimeLight limelight;

    public TurretPitch pitch;
    public TurretFlywheel flywheel;

    public LaunchTrajectory launchTrajectory = new LaunchTrajectory(0, 0);

    private Supplier<Pose2d> odemetry;
    private Vector2d hubPosition;

    public Turret(Drivetrain drivetrain){
        this.limelight = new LimeLight();
        this.pitch = new TurretPitch();
        this.flywheel = new TurretFlywheel();

        this.odemetry = () -> drivetrain.getPose();
        double cosHeading = this.odemetry.get().getRotation().getCos();
        double sinHeading = this.odemetry.get().getRotation().getSin();
        this.hubPosition = new Vector2d(
            this.odemetry.get().getX() + cosHeading * Constants.Turret.Physics.kUpperHubRadius,
            this.odemetry.get().getY() + sinHeading * Constants.Turret.Physics.kUpperHubRadius
        );
    }

    @Override
    public void periodic(){
        if(limelight.hasTarget()){
            double angle = this.odemetry.get().getRotation().getRadians() + (this.limelight.yawOffset() * Math.PI / 180);
            double distance = LaunchTrajectory.estimateDistance(this.limelight.pitchOffset());
            this.hubPosition = new Vector2d(
                this.odemetry.get().getX() + Math.cos(angle) * distance,
                this.odemetry.get().getY() + Math.sin(angle) * distance
            );
        }
    }

    private double angleToHub(){
        double deltaX = hubPosition.x - this.odemetry.get().getX();
        double deltaY = hubPosition.y - this.odemetry.get().getY();
        return Math.atan2(deltaY, deltaX);
    }

    public double getTurnSetpoint(){
        // do stuff
        return 0;
    }
}