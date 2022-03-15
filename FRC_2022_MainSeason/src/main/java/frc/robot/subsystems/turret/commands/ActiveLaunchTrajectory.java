package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;

public class ActiveLaunchTrajectory extends CommandBase {
    private Turret turret;

    public ActiveLaunchTrajectory(Turret turret){
        this.turret = turret;
        this.turret.launchTrajectory = new LaunchTrajectory(0, 0);
        
        this.addRequirements(this.turret);
    }

    @Override
    public void execute(){
        this.turret.pitch.setPosition(this.turret.launchTrajectory.theta);

        // force stop if target speed is 0
        if(this.turret.launchTrajectory.speed == 0){
            this.turret.flywheel.setPower(0);
        } else {
            this.turret.flywheel.setVelocity(this.turret.launchTrajectory.speed);
        }        
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
