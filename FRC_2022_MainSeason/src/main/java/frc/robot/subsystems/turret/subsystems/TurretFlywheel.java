package frc.robot.subsystems.turret.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class TurretFlywheel extends SubsystemBase {
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

        // this.encoderA.setVelocityConversionFactor(?)
        // this.encoderB.setVelocityConversionFactor(?)
        // maybe 2πr

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

        SmartDashboard.putNumber("flywheel kP", Constants.Turret.TunedCoefficients.FlywheelPID.kP);
        SmartDashboard.putNumber("flywheel kI", Constants.Turret.TunedCoefficients.FlywheelPID.kI);
        SmartDashboard.putNumber("flywheel kD", Constants.Turret.TunedCoefficients.FlywheelPID.kD);
        SmartDashboard.putNumber("flywheel kFF", Constants.Turret.TunedCoefficients.FlywheelPID.kFF);
    }

    // testing
    @Override
    public void periodic(){
        double p = SmartDashboard.getNumber("flywheel kP", 0.0);
        double i = SmartDashboard.getNumber("flywheel kI", 0.0);
        double d = SmartDashboard.getNumber("flywheel kD", 0.0);
        double ff = SmartDashboard.getNumber("flywheel kFF", 0.0);

        if(Constants.Turret.TunedCoefficients.FlywheelPID.kP != p){
            Constants.Turret.TunedCoefficients.FlywheelPID.kP = p;
            this.PIDControllerA.setP(p);
            this.PIDControllerB.setP(p);
        }
        if(Constants.Turret.TunedCoefficients.FlywheelPID.kI != i){
            Constants.Turret.TunedCoefficients.FlywheelPID.kI = i;
            this.PIDControllerA.setP(i);
            this.PIDControllerB.setP(i);
        }
        if(Constants.Turret.TunedCoefficients.FlywheelPID.kD != d){
            Constants.Turret.TunedCoefficients.FlywheelPID.kD = d;
            this.PIDControllerA.setP(d);
            this.PIDControllerB.setP(d);
        }
        if(Constants.Turret.TunedCoefficients.FlywheelPID.kFF != ff){
            Constants.Turret.TunedCoefficients.FlywheelPID.kFF = ff;
            this.PIDControllerA.setP(ff);
            this.PIDControllerB.setP(ff);
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

    public void setVelocitySetpoint(double velocity){
        // ! - wait until turret flywheel motor works
        // this.PIDControllerA.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        // this.PIDControllerB.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    public void setVelocitySetpointTesting(double velocity){
        this.PIDControllerA.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        this.PIDControllerB.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    public boolean isAtRest(){
        return this.getVelocity() < Constants.Turret.TunedCoefficients.FlywheelPID.kStoppedVelocity;
    }

    public boolean isUpToSpeed(double launchTrajectorySpeed){
        return Math.abs(this.getVelocity() - launchTrajectorySpeed) < Constants.Turret.TunedCoefficients.FlywheelPID.kErrorThreshold;
    }

    public void setFlywheelSpeed(double velocity) {
        this.motorA.set(velocity);
        this.motorB.set(-velocity);
    }
}