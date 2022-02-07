package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;

public class MatchHeadingYaw extends PIDCommand {
    private Turret turret;

    // Boolean representation of the sign coefficient applied to the seek adjustment
    private boolean seekDirection = true;

    public MatchHeadingYaw(Turret turret){
        super(
            new PIDController(
                Constants.Turret.TunedCoefficients.YawPID.p,
                Constants.Turret.TunedCoefficients.YawPID.i,
                Constants.Turret.TunedCoefficients.YawPID.d
            ),
            () -> {
                return turret.getYaw();
            },
            () -> 0.0,
            (output) -> {
                turret.setYawPower(output);
            }
        );
        this.getController().setTolerance(Constants.Turret.TunedCoefficients.YawPID.errorThreshold);
        this.turret = turret;
    }

    @Override
    public void execute(){
        if(this.turret.limelight.hasTarget()){
            double targetYaw = this.turret.getYaw() + this.turret.limelight.yawOffset();
            if(
                targetYaw < Constants.Turret.TunedCoefficients.YawPID.minSafeAngle ||
                targetYaw > Constants.Turret.TunedCoefficients.YawPID.maxSafeAngle
            ){
                this.seekDirection = targetYaw < Constants.Turret.TunedCoefficients.YawPID.minSafeAngle;
            } else {
                this.m_useOutput.accept(
                    this.m_controller.calculate(
                        this.m_measurement.getAsDouble(),
                        this.m_setpoint.getAsDouble()
                    )
                );
                return;
            }
        }
        this.m_useOutput.accept(
            this.seekDirection ?
            Constants.Turret.TunedCoefficients.YawPID.seekAdjustment :
            -Constants.Turret.TunedCoefficients.YawPID.seekAdjustment
        );
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
