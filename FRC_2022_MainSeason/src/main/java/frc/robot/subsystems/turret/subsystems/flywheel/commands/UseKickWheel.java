package frc.robot.subsystems.turret.subsystems.flywheel.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.subsystems.flywheel.TurretFlywheel;

public class UseKickWheel extends CommandBase {
    private TurretFlywheel flywheel;

    public UseKickWheel(TurretFlywheel flywheel){
        this.flywheel = flywheel;
    }

    @Override
    public void execute(){
        // TODO
    }

    @Override
    public void end(boolean interrupted){
        
    }

    @Override
    public boolean isFinished(){
        // TODO
        return false;
    }
}
