package frc.robot.subsystems.turret.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class TurretFlywheel extends SubsystemBase {
    private CANSparkMax flywheelMotorA;
    private CANSparkMax flywheelMotorB;
    private CANSparkMax kickWheelMotor;

    private RelativeEncoder flyEncoderA;
    private RelativeEncoder flyEncoderB;

    public TurretFlywheel(){
        flywheelMotorA = new CANSparkMax(Constants.Turret.Ports.kFlywheelMotorA, CANSparkMax.MotorType.kBrushless);
        flywheelMotorB = new CANSparkMax(Constants.Turret.Ports.kFlywheelMotorB, CANSparkMax.MotorType.kBrushless);
        kickWheelMotor = new CANSparkMax(Constants.Turret.Ports.kKickWheel, CANSparkMax.MotorType.kBrushless);

        flyEncoderA = flywheelMotorA.getEncoder();
        flyEncoderB = flywheelMotorB.getEncoder();

        flywheelMotorB.follow(flywheelMotorA);
        
        flywheelMotorA.setInverted(false);
        flywheelMotorB.setInverted(true);

    }

    public double getVelocity(){
        double speedA = this.flyEncoderA.getVelocity(); //should return in ticks per second
        double speedB = this.flyEncoderB.getVelocity(); //should return in ticks per second
        return 0.5 * (speedA + speedB);
    }

    public void setPower(double power){
        flywheelMotorA.set(power);
        flywheelMotorB.set(power);
    }

    public boolean isUpToSpeed(double launchTrajectorySpeed){
        return Math.abs(this.getVelocity() - launchTrajectorySpeed) < Constants.Turret.TunedCoefficients.FlywheelPID.kErrorThreshold;
    }

}