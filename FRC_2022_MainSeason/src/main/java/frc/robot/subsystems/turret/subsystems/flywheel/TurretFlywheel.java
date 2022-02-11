package frc.robot.subsystems.turret.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

public class TurretFlywheel extends SubsystemBase {
    private CANSparkMax flywheelMotorA;
    private CANSparkMax flywheelMotorB;
    private CANSparkMax kickWheelMotor;
    private RelativeEncoder flywheelEncoderA;
    private RelativeEncoder flywheelEncoderB;

    public TurretFlywheel(){
        this.flywheelMotorA = new CANSparkMax(Constants.Turret.Ports.kFlywheelMotorA, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.flywheelMotorB = new CANSparkMax(Constants.Turret.Ports.kFlywheelMotorB, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.kickWheelMotor = new CANSparkMax(Constants.Turret.Ports.kKickWheel, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.flywheelEncoderA = this.flywheelMotorA.getEncoder();
        this.flywheelEncoderB = this.flywheelMotorB.getEncoder();
    }

    public double getVelocity(){
        double speedA = this.flywheelEncoderA.getVelocity(); //should return in ticks per second
        double speedB = this.flywheelEncoderB.getVelocity(); //should return in ticks per second
        return 0.5 * (speedA + speedB);
    }

    public void setPower(double power){
        
    }

    public boolean isUpToSpeed(double launchTrajectorySpeed){
        return Math.abs(this.getVelocity() - launchTrajectorySpeed) < Constants.Turret.TunedCoefficients.FlywheelPID.kErrorThreshold;
    }
}