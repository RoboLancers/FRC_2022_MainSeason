package frc.robot.subsystems.turret.subsystems;

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
    
    private RelativeEncoder encoder;

    private SparkMaxPIDController PIDController;

    public TurretFlywheel(){
        this.motorA = new CANSparkMax(Constants.Turret.Ports.kFlywheelMotorA, CANSparkMax.MotorType.kBrushless);
        this.motorB = new CANSparkMax(Constants.Turret.Ports.kFlywheelMotorB, CANSparkMax.MotorType.kBrushless);

        this.motorA.setInverted(false);

        this.motorA.setIdleMode(IdleMode.kCoast);
        this.motorB.setIdleMode(IdleMode.kCoast);

        this.motorB.follow(this.motorA, true);
        
        this.encoder = this.motorA.getEncoder();

        this.encoder.setPosition(0.0);

        this.encoder.setVelocityConversionFactor(1);

        this.PIDController = this.motorA.getPIDController();

        this.PIDController.setP(Constants.Turret.Flywheel.kP);
        this.PIDController.setI(Constants.Turret.Flywheel.kI);
        this.PIDController.setD(Constants.Turret.Flywheel.kD);
        this.PIDController.setD(Constants.Turret.Flywheel.kD);
        this.PIDController.setFF(Constants.Turret.Flywheel.kFF);
        this.PIDController.setOutputRange(
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

        this.PIDController.setP(kP);
        this.PIDController.setI(kI);
        this.PIDController.setD(kD);
        this.PIDController.setFF(kFF);
    }

    public void setVelocity(double velocity){
        this.PIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    public void setPower(double power){
        this.motorA.set(power);
    }

    public double getVelocity(){
        return this.encoder.getVelocity();
    }

    public double getCurrent(){
        // TODO
        return 0;
    }

    public boolean isAtRest(){
        return Math.abs(this.getVelocity()) < Constants.Turret.Flywheel.kErrorThreshold;
    }

    public boolean isUpToSpeed(double velocitySetpoint){
        return Math.abs(this.getVelocity() - velocitySetpoint) < Constants.Turret.Flywheel.kErrorThreshold;
    }
}