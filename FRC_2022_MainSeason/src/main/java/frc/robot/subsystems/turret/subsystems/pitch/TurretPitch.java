package frc.robot.subsystems.turret.subsystems.pitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TurretPitch extends SubsystemBase {
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController smartMotionController;

    private DigitalInput homingSwitch;

    public TurretPitch(){
        this.motor = new CANSparkMax(Constants.Turret.Ports.kPitchMotor, CANSparkMax.MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();
        this.smartMotionController = this.motor.getPIDController();

        this.smartMotionController.setP(Constants.Turret.TunedCoefficients.PitchPID.kP);
        this.smartMotionController.setI(Constants.Turret.TunedCoefficients.PitchPID.kI);
        this.smartMotionController.setD(Constants.Turret.TunedCoefficients.PitchPID.kD);
        this.smartMotionController.setD(Constants.Turret.TunedCoefficients.PitchPID.kD);
        this.smartMotionController.setIZone(Constants.Turret.TunedCoefficients.PitchPID.kIz);
        this.smartMotionController.setFF(Constants.Turret.TunedCoefficients.PitchPID.kFF);
        this.smartMotionController.setOutputRange(
            -Constants.Turret.TunedCoefficients.PitchPID.kMaxAbsoluteOutput,
            Constants.Turret.TunedCoefficients.PitchPID.kMaxAbsoluteOutput
        );

        // TODO: implement the rest of this on a computer that has the api

        this.homingSwitch = new DigitalInput(Constants.Turret.Ports.kYawLimitSwitch);
    }

    public double getVelocity(){
        // TODO
        return 0.0;
    }

    public double getPosition(){
        return 0.0;
    }

    public void setPower(double power){
        // TODO
    }

    public boolean isAligned(double launchTrajectoryTheta){
        return Math.abs((this.getPosition() + Constants.Turret.PhysicsInfo.kPitchMountAngle) - launchTrajectoryTheta) < Constants.Turret.TunedCoefficients.PitchPID.kErrorThreshold;
    }
}
