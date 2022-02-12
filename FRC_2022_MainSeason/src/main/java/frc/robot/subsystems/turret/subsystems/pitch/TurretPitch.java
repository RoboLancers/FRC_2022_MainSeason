package frc.robot.subsystems.turret.subsystems.pitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class TurretPitch extends SubsystemBase {
    private CANSparkMax motor;
    private RelativeEncoder encoder;

    private DigitalInput homingSwitch;

    public TurretPitch(){
        this.motor = new CANSparkMax(Constants.Turret.Ports.kPitchMotor, CANSparkMax.MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();

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
