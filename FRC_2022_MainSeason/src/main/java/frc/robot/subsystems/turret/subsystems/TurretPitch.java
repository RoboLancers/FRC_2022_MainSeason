package frc.robot.subsystems.turret.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

public class TurretPitch extends SubsystemBase {
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController PIDController;

    private DigitalInput homingSwitch;
    private Trigger homingTrigger;

    public TurretPitch(){
        this.motor = new CANSparkMax(Constants.Turret.Ports.kPitchMotor, CANSparkMax.MotorType.kBrushless);
        this.motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Turret.Pitch.kMinSafeAngle);
        this.motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Turret.Pitch.kMaxSafeAngle);

        this.motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        this.motor.enableSoftLimit(SoftLimitDirection.kForward, true);

        this.encoder = this.motor.getEncoder();
        this.encoder.setPositionConversionFactor(Constants.Turret.Pitch.kGearRatio);
        this.resetEncoder();

        this.PIDController = this.motor.getPIDController();
        this.PIDController.setP(Constants.Turret.Pitch.kP);
        this.PIDController.setI(Constants.Turret.Pitch.kI);
        this.PIDController.setD(Constants.Turret.Pitch.kD);
        this.PIDController.setFF(Constants.Turret.Pitch.kFF);
        this.PIDController.setOutputRange(
            -Constants.Turret.Pitch.kMaxAbsoluteVoltage,
            Constants.Turret.Pitch.kMaxAbsoluteVoltage
        );

        this.homingSwitch = new DigitalInput(Constants.Turret.Ports.kPitchLimitSwitch);
        this.homingTrigger = new Trigger(this::limitSwitchTriggered);
        this.homingTrigger.whenActive(new InstantCommand(() -> {
            this.resetEncoder();
        }, this));

        SmartDashboard.putNumber("Pitch kP", SmartDashboard.getNumber("Pitch kP", Constants.Turret.Pitch.kP));
        SmartDashboard.putNumber("Pitch kI", SmartDashboard.getNumber("Pitch kI", Constants.Turret.Pitch.kI));
        SmartDashboard.putNumber("Pitch kD", SmartDashboard.getNumber("Pitch kD", Constants.Turret.Pitch.kD));
        SmartDashboard.putNumber("Pitch kFF", SmartDashboard.getNumber("Pitch kFF", Constants.Turret.Pitch.kFF));
    }

    @Override
    public void periodic(){
        double kP = SmartDashboard.getNumber("Pitch kP", 0);
        double kI = SmartDashboard.getNumber("Pitch kI", 0);
        double kD = SmartDashboard.getNumber("Pitch kD", 0);
        double kFF = SmartDashboard.getNumber("Pitch kFF", 0);

        this.PIDController.setP(kP);
        this.PIDController.setI(kI);
        this.PIDController.setD(kD);
        this.PIDController.setFF(kFF);
    }

    public void setPosition(double position){
        this.PIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void enableSoftLimits(boolean enableState){
        this.motor.enableSoftLimit(SoftLimitDirection.kReverse, enableState);
        this.motor.enableSoftLimit(SoftLimitDirection.kForward, enableState);
    }

    public void setMotorPower(double power){
        this.motor.set(power);
    }

    public void resetEncoder(){
        this.encoder.setPosition(0);
    }

    public boolean limitSwitchTriggered(){
        return !this.homingSwitch.get();
    }

    public double getPosition(){
        return this.encoder.getPosition();
    }

    public boolean isAtZero(){
        return Math.abs(this.getPosition()) < Constants.Turret.Pitch.kErrorThreshold;
    }

    public boolean isAligned(double positionSetpoint){
        return Math.abs(this.getPosition() - positionSetpoint) < Constants.Turret.Pitch.kErrorThreshold;
    }
}
