package frc.robot.subsystems.turret.subsystems.yaw;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.misc.LimeLight;
import frc.robot.subsystems.turret.Turret;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class BetterYaw extends SubsystemBase {
    public LimeLight limelight;

    public BetterYaw(Turret turret, Drivetrain driveTrain) {
        this.limelight = new LimeLight();
        
    }

    public boolean isAligned(){
        return (this.limelight.hasTarget());
    }
}


// TODO: Roller stuff: make this an inline command