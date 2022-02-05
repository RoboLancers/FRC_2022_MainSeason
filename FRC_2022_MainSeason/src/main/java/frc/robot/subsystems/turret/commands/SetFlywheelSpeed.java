package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.util.SparkMaxWrapper;

// Set flywheel speed using pid
public class SetFlywheelSpeed extends PIDCommand {
    private static double velocityErrorThreshold = 50.0;

    public SetFlywheelSpeed(double targetSpeed, SparkMaxWrapper flywheelMotorA, SparkMaxWrapper flywheelMotorB){
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
            () -> targetSpeed,
            (output) -> {
                flywheelMotorA.setPower(output);
                flywheelMotorB.setPower(output);
            }
        );
        this.getController().setTolerance(velocityErrorThreshold);
    }

    @Override
    public void end(boolean interrupted){};

    @Override
    public boolean isFinished(){
        return this.getController().atSetpoint();
    }
}
