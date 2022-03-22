package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class ActivePitch extends CommandBase {
    private Turret turret;

    public ActivePitch(Turret turret){
        this.turret = turret;

        this.addRequirements(this.turret.pitch);
    }

    @Override
    public void execute(){
        this.turret.pitch.setPosition(this.turret.launchTrajectory.theta);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
