package frc.robot.subsystems.turret.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class TurretPitch extends SubsystemBase {
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController PIDController;

    private DigitalInput homingSwitch;

    public TurretPitch(){
        this.motor = new CANSparkMax(Constants.Turret.Ports.kPitchMotor, CANSparkMax.MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();

        this.PIDController = this.motor.getPIDController();

        this.PIDController.setP(Constants.Turret.TunedCoefficients.PitchPID.kP);
        this.PIDController.setI(Constants.Turret.TunedCoefficients.PitchPID.kI);
        this.PIDController.setD(Constants.Turret.TunedCoefficients.PitchPID.kD);
        this.PIDController.setD(Constants.Turret.TunedCoefficients.PitchPID.kD);
        this.PIDController.setIZone(Constants.Turret.TunedCoefficients.PitchPID.kIz);
        this.PIDController.setFF(Constants.Turret.TunedCoefficients.PitchPID.kFF);
        this.PIDController.setOutputRange(
            -Constants.Turret.TunedCoefficients.PitchPID.kMaxAbsoluteOutput,
            Constants.Turret.TunedCoefficients.PitchPID.kMaxAbsoluteOutput
        );

        this.homingSwitch = new DigitalInput(Constants.Turret.Ports.kYawLimitSwitch);
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

    public boolean isAligned(double launchTrajectoryTheta){
        return Math.abs((this.getPosition() + Constants.Turret.PhysicsInfo.kPitchMountAngle) - launchTrajectoryTheta) < Constants.Turret.TunedCoefficients.PitchPID.kErrorThreshold;
    }
}
