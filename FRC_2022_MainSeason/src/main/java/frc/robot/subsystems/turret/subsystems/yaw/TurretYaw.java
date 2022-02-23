package frc.robot.subsystems.turret.subsystems.yaw;

import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.misc.LimeLight;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.subsystems.yaw.commands.ActiveLaunchTrajectory;
import frc.robot.subsystems.turret.subsystems.yaw.commands.MatchHeadingYaw;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TurretYaw extends SubsystemBase {
    private Consumer<LaunchTrajectory> onLaunchTrajectoryUpdate;

    public LimeLight limelight;

    private CANSparkMax motor;
    private RelativeEncoder encoder;

    private DigitalInput homingSwitch;

    public TurretYaw(Consumer<LaunchTrajectory> onLaunchTrajectoryUpdate){
        this.onLaunchTrajectoryUpdate = onLaunchTrajectoryUpdate;

        this.limelight = new LimeLight();

        this.motor = new CANSparkMax(Constants.Turret.Ports.kYawMotor, MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();

        this.homingSwitch = new DigitalInput(Constants.Turret.Ports.kYawLimitSwitch);

        // TODO: use default commands or at least find a way to make it end after shooting phase of the game
        new PerpetualCommand(new ActiveLaunchTrajectory(this));
        new PerpetualCommand(new MatchHeadingYaw(this));
    }

    public void updateLaunchTrajectory(LaunchTrajectory newLaunchTrajectory){
        this.onLaunchTrajectoryUpdate.accept(newLaunchTrajectory);
    }

    public double getVelocity(){
        // TODO
        return 0.0;
    }

    public double getPosition(){
        // TODO
        return 0.0;
    }

    public void setPower(double power){
        // TODO
    }

    public boolean isAligned(){
        return (
            this.limelight.hasTarget() &&
            Math.abs(this.limelight.yawOffset()) < Constants.Turret.TunedCoefficients.YawPID.kErrorThreshold
        );
    }
}
