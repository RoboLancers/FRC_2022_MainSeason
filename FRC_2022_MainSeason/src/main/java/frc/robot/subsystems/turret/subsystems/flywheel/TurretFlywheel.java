package frc.robot.subsystems.turret.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.qualcomm.robotcore.hardware;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TurretFlywheel extends SubsystemBase {
    private CANSparkMax flywheelMotorA;
    private CANSparkMax flywheelMotorB;
    private CANSparkMax kickWheelMotor;
    private CANEncoder flywheelEncoderA;

    public TurretFlywheel(){
        this.flywheelMotor = new CANSparkMax(CANSparkMaxLowLevel.MotorType.kBrushless, Constants.Turret.Ports.kFlywheelMotorA);
        this.flywheelMotor = new CANSparkMax(CANSparkMaxLowLevel.MotorType.kBrushless, Constants.Turret.Ports.kFlywheelMotorB);
        this.kickWheelMotor = new CANSparkMax(CANSparkMaxLowLevel.MotorType.kBrushless, Constants.Turret.Ports.kKickWheel);
        this.flywheelEncoderA = this.flywheelEncoderA.getEncoder();
        this.flywheelEncoderB = this.flywheelEncoderB.getEncoder();
    }

    public double getVelocity(){
        double speedA = this.flyEnocderA.getVelocity(); //should return in ticks per second
        double speedB = this.flyEnocderB.getVelocity(); //should return in ticks per second
        return 0.5 * (speedA + speedB);
    }

    public void setPower(double power){
        
    }

    public boolean isUpToSpeed(double launchTrajectorySpeed){
        return Math.abs(this.getVelocity() - launchTrajectorySpeed) < Constants.Turret.TunedCoefficients.FlywheelPID.kErrorThreshold;
    }
}