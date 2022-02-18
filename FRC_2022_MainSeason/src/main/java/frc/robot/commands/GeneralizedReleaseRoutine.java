package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.commands.ShootBall;
import frc.robot.subsystems.turret.Turret;

public class GeneralizedReleaseRoutine extends CommandBase {
    private Indexer indexer;
    private Turret turret;

    public GeneralizedReleaseRoutine(Indexer indexer, Turret turret){
        this.indexer = indexer;
        this.turret = turret;
    };

    @Override
    public void execute(){
        if(turret.inShootingRange()) {
            ShootBall ballShoot = new ShootBall(indexer);
        }
    }
}
