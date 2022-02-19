package frc.robot.subsystems.turret.subsystems.yaw.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.turret.subsystems.yaw.TurretYaw;

public class MatchHeadingYaw extends PIDCommand {
    private TurretYaw turretYaw;

    // Boolean representation of the sign coefficient applied to the seek adjustment
    private boolean seekDirection = true;

    public MatchHeadingYaw(TurretYaw turretYaw){
        super(
            new PIDController(
                Constants.Turret.TunedCoefficients.YawPID.kP,
                Constants.Turret.TunedCoefficients.YawPID.kI,
                Constants.Turret.TunedCoefficients.YawPID.kD
            ),
            () -> {
                return turretYaw.getPosition();
            },
            () -> 0.0,
            (output) -> {
                turretYaw.setPower(output);
            }
        );
        this.getController().setTolerance(Constants.Turret.TunedCoefficients.YawPID.kErrorThreshold);
        this.turretYaw = turretYaw;
    }

    @Override
    public void execute(){
        if(this.turretYaw.limelight.hasTarget()){
            double targetYaw = this.turretYaw.getPosition() + this.turretYaw.limelight.yawOffset();
            if(
                targetYaw < Constants.Turret.TunedCoefficients.YawPID.kMinSafeAngle ||
                targetYaw > Constants.Turret.TunedCoefficients.YawPID.kMaxSafeAngle
            ){
                this.seekDirection = targetYaw < Constants.Turret.TunedCoefficients.YawPID.kMinSafeAngle;
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
            Constants.Turret.TunedCoefficients.YawPID.kSeekAdjustment :
            -Constants.Turret.TunedCoefficients.YawPID.kSeekAdjustment
        );
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
