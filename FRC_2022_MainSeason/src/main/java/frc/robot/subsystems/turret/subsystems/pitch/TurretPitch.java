package frc.robot.subsystems.turret.subsystems.pitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.qualcomm.robotcore.hardware;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TurretPitch extends SubsystemBase {
    private CANSparkMax motor;
    private CANEncoder encoder;

    private DigitalInput homingSwitch;

    public TurretPitch(){
        this.motor = new CANSparkMax(CANSparkMax.MotorType.kBrushless, Constants.Turret.Ports.kPitchMotor);
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
