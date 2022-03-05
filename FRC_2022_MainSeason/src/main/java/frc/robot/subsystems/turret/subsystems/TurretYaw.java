package frc.robot.subsystems.turret.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.misc.LimeLight;
import frc.robot.subsystems.turret.Turret;

public class TurretYaw extends SubsystemBase {
    public LimeLight limelight;

    public TurretYaw(Turret turret, Drivetrain driveTrain) {
        this.limelight = new LimeLight();
    }

    public boolean isAligned(){
        return (this.limelight.hasTarget());
    }
}