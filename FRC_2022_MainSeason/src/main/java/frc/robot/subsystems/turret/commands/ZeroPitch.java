package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;

public class ZeroPitch extends CommandBase {
    private Turret turret;

    public ZeroPitch(Turret turret){
        this.turret = turret;

        addRequirements(this.turret);
    };

    @Override
    public void execute(){
        this.turret.pitch.positionSetpoint = this.turret.pitch.positionSetpoint - Constants.Turret.Pitch.kZeroAdjustment;
    }

    @Override
    public boolean isFinished(){
        return !this.turret.pitch.homingSwitch.get();
    }
}