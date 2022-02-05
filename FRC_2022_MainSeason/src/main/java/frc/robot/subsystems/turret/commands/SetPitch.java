package frc.robot.subsystems.turret.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.util.SparkMaxWrapper;

// Set pitch using pid
public class SetPitch extends PIDCommand {
    public SetPitch(DoubleSupplier targetYaw, SparkMaxWrapper pitchMotor){
        super(
            new PIDController(
                Constants.Turret.TunedCoefficients.PitchPID.p,
                Constants.Turret.TunedCoefficients.PitchPID.i,
                Constants.Turret.TunedCoefficients.PitchPID.d
            ),
            () -> {
                return pitchMotor.getPosition();
            },
            targetYaw,
            (output) -> {
                pitchMotor.setPower(output);
            }
        );
        this.getController().setTolerance(Constants.Turret.TunedCoefficients.PitchPID.errorThreshold);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
