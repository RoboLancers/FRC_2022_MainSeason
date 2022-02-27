package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class ZeroAndDisable extends CommandBase {
    private Turret turret;

    public ZeroAndDisable(Turret turret){
        this.turret = turret;

        this.turret.yaw.setPositionSetpoint(0);
        this.turret.pitch.setPositionSetpoint(0);
        this.turret.flywheel.setVelocitySetpoint(0);

        addRequirements(turret);
    };

    @Override
    public void end(boolean interrupted){
        this.turret.inHangMode = true;
    }

    @Override
    public boolean isFinished(){
        return (
            this.turret.yaw.isAtZero() &&
            this.turret.pitch.isAtZero() &&
            this.turret.flywheel.isAtRest()
        );
    }
}
