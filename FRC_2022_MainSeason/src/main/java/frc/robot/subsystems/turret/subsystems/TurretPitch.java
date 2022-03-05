package frc.robot.subsystems.turret.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class TurretPitch extends SubsystemBase {
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController PIDController;

    private DigitalInput homingSwitch;
    private Trigger homingTrigger;

    public TurretPitch(){
        this.motor = new CANSparkMax(Constants.Turret.Ports.kPitchMotor, CANSparkMax.MotorType.kBrushless);

        this.encoder = this.motor.getEncoder();
        this.encoder.setPositionConversionFactor(Constants.Turret.TunedCoefficients.PitchPID.ratio);
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
        this.homingTrigger = new Trigger(this.homingSwitch::get);
        this.homingTrigger.whenActive(new RunCommand(() -> {
            this.encoder.setPosition(0);
        }, this));

        SmartDashboard.putNumber("pitch kP", Constants.Turret.TunedCoefficients.PitchPID.kP);
        SmartDashboard.putNumber("pitch kI", Constants.Turret.TunedCoefficients.PitchPID.kI);
        SmartDashboard.putNumber("pitch kD", Constants.Turret.TunedCoefficients.PitchPID.kD);
        SmartDashboard.putNumber("pitch kFF", Constants.Turret.TunedCoefficients.PitchPID.kFF);
    }

    // testing
    @Override
    public void periodic(){

        SmartDashboard.putBoolean("PitchSwitch", this.homingSwitch.get());

        double p = SmartDashboard.getNumber("pitch kP", 0.0);
        double i = SmartDashboard.getNumber("pitch kI", 0.0);
        double d = SmartDashboard.getNumber("pitch kD", 0.0);
        double ff = SmartDashboard.getNumber("pitch kFF", 0.0);

        if(Constants.Turret.TunedCoefficients.PitchPID.kP != p){
            Constants.Turret.TunedCoefficients.PitchPID.kP = p;
            this.PIDController.setP(p);
        }
        if(Constants.Turret.TunedCoefficients.PitchPID.kI != i){
            Constants.Turret.TunedCoefficients.PitchPID.kI = i;
            this.PIDController.setP(i);
        }
        if(Constants.Turret.TunedCoefficients.PitchPID.kD != d){
            Constants.Turret.TunedCoefficients.PitchPID.kD = d;
            this.PIDController.setP(d);
        }
        if(Constants.Turret.TunedCoefficients.PitchPID.kFF != ff){
            Constants.Turret.TunedCoefficients.PitchPID.kFF = ff;
            this.PIDController.setP(ff);
        }
    }

    private double getPosition(){
        return this.encoder.getPosition();
    }

    public void setPositionSetpoint(double position){
        // ! - wait until turret pitch motor works
        // this.PIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void setPositionSetpointTesting(double position){
        this.PIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public boolean isAtZero(){
        return Math.abs(this.getPosition()) < Constants.Turret.TunedCoefficients.PitchPID.kStoppedPosition;
    }

    public boolean isAligned(double launchTrajectoryTheta){
        return Math.abs(this.getPosition() + Constants.Turret.PhysicsInfo.kPitchMountAngle - launchTrajectoryTheta) < Constants.Turret.TunedCoefficients.PitchPID.kErrorThreshold;
    }
}
