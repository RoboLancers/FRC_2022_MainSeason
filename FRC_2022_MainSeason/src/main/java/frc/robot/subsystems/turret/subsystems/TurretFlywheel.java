package frc.robot.subsystems.turret.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

public class TurretFlywheel extends SubsystemBase {
    private CANSparkMax motorA;
    private CANSparkMax motorB;
    
    private RelativeEncoder encoderA;
    private RelativeEncoder encoderB;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.18017, 0.12747, 0.0038965);

    private SparkMaxPIDController PIDControllerA;
    private SparkMaxPIDController PIDControllerB;

    public TurretFlywheel(){
        this.motorA = new CANSparkMax(Constants.Turret.Ports.kFlywheelMotorA, CANSparkMax.MotorType.kBrushless);
        this.motorB = new CANSparkMax(Constants.Turret.Ports.kFlywheelMotorB, CANSparkMax.MotorType.kBrushless);

        this.motorA.setInverted(false);
        this.motorB.setInverted(true);

        this.motorB.follow(this.motorA);
        
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

        SmartDashboard.putNumber("Flywheel kP", SmartDashboard.getNumber("Flywheel kP", Constants.Turret.Flywheel.kP));
        SmartDashboard.putNumber("Flywheel kI", SmartDashboard.getNumber("Flywheel kI", Constants.Turret.Flywheel.kI));
        SmartDashboard.putNumber("Flywheel kD", SmartDashboard.getNumber("Flywheel kD", Constants.Turret.Flywheel.kD));
        SmartDashboard.putNumber("Flywheel kFF", SmartDashboard.getNumber("Flywheel kFF", Constants.Turret.Flywheel.kFF));
    }

    @Override
    public void periodic(){
        double kP = SmartDashboard.getNumber("Flywheel kP", 0);
        double kI = SmartDashboard.getNumber("Flywheel kI", 0);
        double kD = SmartDashboard.getNumber("Flywheel kD", 0);
        double kFF = SmartDashboard.getNumber("Flywheel kFF", 0);

        this.PIDControllerA.setP(kP);
        this.PIDControllerA.setI(kI);
        this.PIDControllerA.setD(kD);
        this.PIDControllerA.setFF(kFF);

        this.PIDControllerB.setP(kP);
        this.PIDControllerB.setI(kI);
        this.PIDControllerB.setD(kD);
        this.PIDControllerB.setFF(kFF);
    }

    public void setVelocity(double velocity){
        this.PIDControllerA.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        this.PIDControllerB.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    public void setPower(double power){
        this.motorA.set(power);
    }

    public double getVelocity(){
        return this.encoderA.getVelocity();
    }

    public double getCurrent(){
        return this.motorA.getOutputCurrent();
    }

    public boolean isAtRest(){
        return Math.abs(this.getVelocity()) < Constants.Turret.Flywheel.kErrorThreshold;
    }

    public boolean isUpToSpeed(double velocitySetpoint){
        return Math.abs(this.getVelocity() - velocitySetpoint) < Constants.Turret.Flywheel.kErrorThreshold;
    }
}