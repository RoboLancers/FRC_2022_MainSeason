package frc.robot.subsystems.turret.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

public class TurretPitch extends SubsystemBase {
    public double positionSetpoint = 0;

    public CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController PIDController;

    public DigitalInput homingSwitch;
    private Trigger homingTrigger;

    public TurretPitch(){
        this.motor = new CANSparkMax(Constants.Turret.Ports.kPitchMotor, CANSparkMax.MotorType.kBrushless);
        this.motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Turret.Pitch.kMinSafeAngle);
        this.motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Turret.Pitch.kMaxSafeAngle);
        this.motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        this.motor.enableSoftLimit(SoftLimitDirection.kForward, true);

        this.encoder = this.motor.getEncoder();
        this.encoder.setPositionConversionFactor(Constants.Turret.Pitch.kGearRatio);
        this.encoder.setPosition(0.0);

        this.PIDController = this.motor.getPIDController();
        this.PIDController.setP(Constants.Turret.Pitch.kP);
        this.PIDController.setI(Constants.Turret.Pitch.kI);
        this.PIDController.setD(Constants.Turret.Pitch.kD);
        this.PIDController.setFF(Constants.Turret.Pitch.kFF);
        this.PIDController.setOutputRange(
            -Constants.Turret.Pitch.kMaxAbsoluteVoltage,
            Constants.Turret.Pitch.kMaxAbsoluteVoltage
        );

        // this is inverted?
        this.homingSwitch = new DigitalInput(Constants.Turret.Ports.kPitchLimitSwitch);
        this.homingTrigger = new Trigger(() -> { return !homingSwitch.get(); });
        this.homingTrigger.whenActive(new RunCommand(() -> {
            this.encoder.setPosition(0);
        }, this));
    }

    @Override
    public void periodic(){
        this.PIDController.setReference(this.positionSetpoint, CANSparkMax.ControlType.kPosition);
    }

    public double getPosition(){
        return this.encoder.getPosition();
    }

    public boolean isAtZero(){
        return Math.abs(this.getPosition()) < Constants.Turret.Pitch.kErrorThreshold;
    }

    public boolean isAligned(){
        return Math.abs(this.getPosition() - this.positionSetpoint) < Constants.Turret.Pitch.kErrorThreshold;
    }
}
