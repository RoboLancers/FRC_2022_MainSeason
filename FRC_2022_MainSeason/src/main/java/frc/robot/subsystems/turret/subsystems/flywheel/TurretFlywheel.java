package frc.robot.subsystems.turret.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;

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

    private boolean hasBeenShot;

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

    public void currentDetectionTrigger() {
        Trigger currentDetectionTrigger = new Trigger() { // trigger is a class that'll let you run a command when the trigger is activated
        @Override
        public boolean get() { // this is the condition to run our command (i.e. the ball is close enough, run the command)
            return ((flywheelMotorA.getBusVoltage() >= prevVoltage + minChange) && (Indexer.hasTwoBalls()));
            }
        };
        // This is an example of command composition.
        currentDetectionTrigger.whenActive(hasBeenShot = true) // This runs the processBall() function once the color sensor is activated
        .withInterrupt(this::)
        .andThen(() -> indexerMotor.set(Constants.Indexer.kStandardIndexerSpeed) // After the command is stopped (i.e. the ball reaches the next sensor) stop the indexer motor
        ));
    }

}