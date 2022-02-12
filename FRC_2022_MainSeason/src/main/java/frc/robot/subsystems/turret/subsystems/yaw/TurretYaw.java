package frc.robot.subsystems.turret.subsystems.yaw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.misc.LimeLight;
import frc.robot.subsystems.turret.LaunchTrajectory;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class TurretYaw extends SubsystemBase {
    private Consumer<LaunchTrajectory> onLaunchTrajectoryUpdate;

    public LimeLight limelight;
    private DigitalInput homingSwitch;

    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController PIDController;

    public TurretYaw(Consumer<LaunchTrajectory> onLaunchTrajectoryUpdate){
        this.onLaunchTrajectoryUpdate = onLaunchTrajectoryUpdate;

        this.limelight = new LimeLight();
        this.homingSwitch = new DigitalInput(Constants.Turret.Ports.kYawLimitSwitch);

        this.motor = new CANSparkMax(Constants.Turret.Ports.kYawMotor, CANSparkMax.MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();

        this.PIDController = this.motor.getPIDController();
        this.PIDController.setP(Constants.Turret.TunedCoefficients.YawPID.kP);
        this.PIDController.setI(Constants.Turret.TunedCoefficients.YawPID.kI);
        this.PIDController.setD(Constants.Turret.TunedCoefficients.YawPID.kD);
        this.PIDController.setFF(Constants.Turret.TunedCoefficients.YawPID.kFF);
        this.PIDController.setOutputRange(
            -Constants.Turret.TunedCoefficients.YawPID.kMaxAbsoluteOutput,
            Constants.Turret.TunedCoefficients.YawPID.kMaxAbsoluteOutput
        );
    }

    public void updateLaunchTrajectory(LaunchTrajectory newLaunchTrajectory){
        this.onLaunchTrajectoryUpdate.accept(newLaunchTrajectory);
    }

    @Override
    public void periodic(){
        if(this.homingSwitch.get()){
            // TODO: reset encoder
        }
    }

    public double getPosition(){
        return this.encoder.getPosition();
    }

    public void setPositionSetpoint(double position){
        this.PIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void setVelocitySetpoint(double position){
        this.PIDController.setReference(position, CANSparkMax.ControlType.kVelocity);
    }

    public boolean isAligned(){
        return (
            this.limelight.hasTarget() &&
            Math.abs(this.limelight.yawOffset()) < Constants.Turret.TunedCoefficients.YawPID.kErrorThreshold
        );
    }
}
