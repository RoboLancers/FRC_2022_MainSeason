package frc.robot.subsystems.turret.subsystems.yaw;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.misc.LimeLight;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.subsystems.yaw.commands.MatchHeadingYaw;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;

public class TurretYaw extends SubsystemBase {
    public LimeLight limelight;

    private DigitalInput homingSwitch;
    private Trigger homingTrigger;

    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController PIDController;

    public TurretYaw(){
        this.limelight = new LimeLight();

        this.homingSwitch = new DigitalInput(Constants.Turret.Ports.kYawLimitSwitch);
        this.homingTrigger = new Trigger(this.homingSwitch::get);
        this.homingTrigger.whenActive(new RunCommand(() -> {
            // TODO: reset encoder
        }, this));

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

    public double getPosition(){
        return this.encoder.getPosition();
    }

    public void setPositionSetpoint(double position){
        this.PIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void setVelocitySetpoint(double velocity){
        this.PIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    public boolean isAligned(){
        return (
            this.limelight.hasTarget() &&
            Math.abs(this.limelight.yawOffset()) < Constants.Turret.TunedCoefficients.YawPID.kErrorThreshold
        );
    }
}
