package frc.robot.subsystems.turret.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.util.SparkMaxWrapper;

// Set flywheel speed using pid
public class SetFlywheelSpeed extends PIDCommand {
    public SetFlywheelSpeed(DoubleSupplier targetSpeed, SparkMaxWrapper flywheelMotorA, SparkMaxWrapper flywheelMotorB){
        // Potentially too naive if there are sizable performance differences between the two motors
        super(
            new PIDController(
                Constants.Turret.TunedCoefficients.FlywheelPID.p,
                Constants.Turret.TunedCoefficients.FlywheelPID.i,
                Constants.Turret.TunedCoefficients.FlywheelPID.d
            ),
            () -> {
                // Measurement source is the average velocity of both motors
                return (flywheelMotorA.getVelocity() + flywheelMotorB.getVelocity()) / 2;
            },
            targetSpeed,
            (output) -> {
                flywheelMotorA.setPower(output);
                flywheelMotorB.setPower(output);
            }
        );
        this.getController().setTolerance(Constants.Turret.TunedCoefficients.FlywheelPID.errorThreshold);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
