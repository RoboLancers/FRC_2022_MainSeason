package frc.robot.subsystems.turret.subsystems.flywheel.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.turret.subsystems.flywheel.TurretFlywheel;

// Set flywheel speed using pid
public class SetFlywheelSpeed extends PIDCommand {
    public SetFlywheelSpeed(DoubleSupplier targetSpeed, TurretFlywheel flywheel){
        // Potentially too naive if there are sizable performance differences between the two motors
        super(
            new PIDController(
                Constants.Turret.TunedCoefficients.FlywheelPID.kP,
                Constants.Turret.TunedCoefficients.FlywheelPID.kI,
                Constants.Turret.TunedCoefficients.FlywheelPID.kD
            ),
            () -> {
                return flywheel.getVelocity();
            },
            targetSpeed,
            (output) -> {
                flywheel.setPower(output);
            }
        );
        this.getController().setTolerance(Constants.Turret.TunedCoefficients.FlywheelPID.kErrorThreshold);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
