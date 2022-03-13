package frc.robot.subsystems.turret.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

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

        this.motorA.setInverted(false);
        this.motorB.setInverted(true);
        
        this.encoderA = this.motorA.getEncoder();
        this.encoderB = this.motorB.getEncoder();

        this.motorA.setIdleMode(IdleMode.kCoast);
        this.motorB.setIdleMode(IdleMode.kCoast);

        this.encoderA.setPosition(0.0);
        this.encoderB.setPosition(0.0);

        this.encoderA.setVelocityConversionFactor(1);
        this.encoderB.setVelocityConversionFactor(1);

        this.PIDControllerA = this.motorA.getPIDController();
        this.PIDControllerB = this.motorB.getPIDController();

        this.PIDControllerA.setP(Constants.Turret.Flywheel.kP);
        this.PIDControllerA.setI(Constants.Turret.Flywheel.kI);
        this.PIDControllerA.setD(Constants.Turret.Flywheel.kD);
        this.PIDControllerA.setD(Constants.Turret.Flywheel.kD);
        this.PIDControllerA.setFF(Constants.Turret.Flywheel.kFF);
        this.PIDControllerA.setOutputRange(
            -Constants.Turret.Flywheel.kMaxAbsoluteVoltage,
            Constants.Turret.Flywheel.kMaxAbsoluteVoltage
        );

        this.PIDControllerB.setP(Constants.Turret.Flywheel.kP);
        this.PIDControllerB.setI(Constants.Turret.Flywheel.kI);
        this.PIDControllerB.setD(Constants.Turret.Flywheel.kD);
        this.PIDControllerB.setD(Constants.Turret.Flywheel.kD);
        this.PIDControllerB.setFF(Constants.Turret.Flywheel.kFF);
        this.PIDControllerB.setOutputRange(
            -Constants.Turret.Flywheel.kMaxAbsoluteVoltage,
            Constants.Turret.Flywheel.kMaxAbsoluteVoltage
        );
    }

    @Override
    public void periodic(){
        this.PIDControllerA.setReference(this.velocitySetpoint, CANSparkMax.ControlType.kVelocity);
        this.PIDControllerB.setReference(this.velocitySetpoint, CANSparkMax.ControlType.kVelocity);
        if(this.velocitySetpoint == 0){
            this.motorA.set(0);
            this.motorB.set(0);
        }
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
        return Math.abs(this.getVelocity()) < Constants.Turret.Flywheel.kErrorThreshold;
    }

    public boolean isUpToSpeed(){
        return Math.abs(this.getVelocity() - this.velocitySetpoint) < Constants.Turret.Flywheel.kErrorThreshold;
    }
}