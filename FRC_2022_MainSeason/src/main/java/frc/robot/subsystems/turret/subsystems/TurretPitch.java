package frc.robot.subsystems.turret.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class TurretPitch extends SubsystemBase {
    public double positionSetpoint = 0;

    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController PIDController;

    public DigitalInput homingSwitch;
    private Trigger homingTrigger;

    public TurretPitch(){
        this.motor = new CANSparkMax(Constants.Turret.Ports.kPitchMotor, CANSparkMax.MotorType.kBrushless);

        this.encoder = this.motor.getEncoder();
        this.encoder.setPositionConversionFactor(Constants.Turret.TunedCoefficients.PitchPID.kGearRatio);
        this.encoder.setPosition(0.0);

        this.PIDController = this.motor.getPIDController();

        this.PIDController.setP(Constants.Turret.TunedCoefficients.PitchPID.kP);
        this.PIDController.setI(Constants.Turret.TunedCoefficients.PitchPID.kI);
        this.PIDController.setD(Constants.Turret.TunedCoefficients.PitchPID.kD);
        this.PIDController.setD(Constants.Turret.TunedCoefficients.PitchPID.kD);
        this.PIDController.setFF(Constants.Turret.TunedCoefficients.PitchPID.kFF);
        this.PIDController.setOutputRange(
            0,
            Constants.Turret.TunedCoefficients.PitchPID.kMaxAbsoluteOutput
        );

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
        return Math.abs(this.getPosition()) < Constants.Turret.TunedCoefficients.PitchPID.kStoppedPosition;
    }

    public boolean isAligned(double launchTrajectoryTheta){
        return Math.abs(this.getPosition() + Constants.Turret.PhysicsInfo.kPitchMountAngle - launchTrajectoryTheta) < Constants.Turret.TunedCoefficients.PitchPID.kErrorThreshold;
    }
}
