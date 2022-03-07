package frc.robot.subsystems.turret.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class TurretFlywheel extends SubsystemBase {
    public double velocitySetpoint = 0;

    private CANSparkMax motorA;
    private CANSparkMax motorB;
    
    private RelativeEncoder encoderA;
    private RelativeEncoder encoderB;

    private SparkMaxPIDController PIDControllerA;
    private SparkMaxPIDController PIDControllerB;

    public TurretFlywheel(){
        this.motorA = new CANSparkMax(Constants.Turret.Ports.kFlywheelMotorA, CANSparkMax.MotorType.kBrushless);
        this.motorB = new CANSparkMax(Constants.Turret.Ports.kFlywheelMotorB, CANSparkMax.MotorType.kBrushless);
        this.encoderA = this.motorA.getEncoder();
        this.encoderB = this.motorB.getEncoder();

        this.encoderA.setPosition(0.0);
        this.encoderB.setPosition(0.0);

        this.encoderA.setVelocityConversionFactor(4 * Math.PI);
        this.encoderB.setVelocityConversionFactor(4 * Math.PI);

        this.PIDControllerA = this.motorA.getPIDController();
        this.PIDControllerB = this.motorB.getPIDController();

        this.PIDControllerA.setP(Constants.Turret.TunedCoefficients.FlywheelPID.kP);
        this.PIDControllerA.setI(Constants.Turret.TunedCoefficients.FlywheelPID.kI);
        this.PIDControllerA.setD(Constants.Turret.TunedCoefficients.FlywheelPID.kD);
        this.PIDControllerA.setD(Constants.Turret.TunedCoefficients.FlywheelPID.kD);
        this.PIDControllerA.setFF(Constants.Turret.TunedCoefficients.FlywheelPID.kFF);
        this.PIDControllerA.setOutputRange(
            -Constants.Turret.TunedCoefficients.FlywheelPID.kMaxAbsoluteOutput,
            Constants.Turret.TunedCoefficients.FlywheelPID.kMaxAbsoluteOutput
        );

        this.PIDControllerB.setP(Constants.Turret.TunedCoefficients.FlywheelPID.kP);
        this.PIDControllerB.setI(Constants.Turret.TunedCoefficients.FlywheelPID.kI);
        this.PIDControllerB.setD(Constants.Turret.TunedCoefficients.FlywheelPID.kD);
        this.PIDControllerB.setD(Constants.Turret.TunedCoefficients.FlywheelPID.kD);
        this.PIDControllerB.setFF(Constants.Turret.TunedCoefficients.FlywheelPID.kFF);
        this.PIDControllerB.setOutputRange(
            -Constants.Turret.TunedCoefficients.FlywheelPID.kMaxAbsoluteOutput,
            Constants.Turret.TunedCoefficients.FlywheelPID.kMaxAbsoluteOutput
        );
    }

    @Override
    public void periodic(){
        this.PIDControllerA.setReference(this.velocitySetpoint, CANSparkMax.ControlType.kVelocity);
        this.PIDControllerB.setReference(this.velocitySetpoint, CANSparkMax.ControlType.kVelocity);
    }

    public double getVelocity(){
        double speedA = this.encoderA.getVelocity();
        double speedB = this.encoderB.getVelocity();
        return 0.5 * (speedA + speedB);
    }

    public double getCurrent(){
        double currentA = this.motorA.getOutputCurrent();
        double currentB = this.motorB.getOutputCurrent();
        return 0.5 * (currentA + currentB);
    }

    public boolean isAtRest(){
        return this.getVelocity() < Constants.Turret.TunedCoefficients.FlywheelPID.kStoppedVelocity;
    }

    public boolean isUpToSpeed(double launchTrajectorySpeed){
        return Math.abs(this.getVelocity() - launchTrajectorySpeed) < Constants.Turret.TunedCoefficients.FlywheelPID.kErrorThreshold;
    }
}