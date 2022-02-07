package frc.robot.subsystems.turret.subsystems.pitch.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.turret.subsystems.pitch.TurretPitch;

// TODO: implement this using smart motion

// Set pitch using pid
public class SetPitch extends PIDCommand {
    public SetPitch(DoubleSupplier targetYaw, TurretPitch pitchController){
        super(
            new PIDController(
                Constants.Turret.TunedCoefficients.PitchPID.kP,
                Constants.Turret.TunedCoefficients.PitchPID.kI,
                Constants.Turret.TunedCoefficients.PitchPID.kD
            ),
            () -> {
                return pitchController.getPosition();
            },
            targetYaw,
            (output) -> {
                pitchController.setPower(output);
            }
        );
        this.getController().setTolerance(Constants.Turret.TunedCoefficients.PitchPID.kErrorThreshold);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
