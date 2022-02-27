package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;
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
        if(this.indexer.ballQueue.size() > 0 && this.turret.inShootingRange() && this.turret.isReadyToShoot()){
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    this.indexer.indexerMotor.set(Constants.Indexer.kIndexerSpeed);
                }),
                new WaitUntilCommand(() -> {
                    return turret.flywheel.getCurrent() < Constants.Turret.TunedCoefficients.FlywheelPID.kCurrentSpikeThreshold;
                }),
                new WaitCommand(Constants.Turret.TunedCoefficients.FlywheelPID.kPostSpikeDelay),
                // ! - this is potentially not the right logic, we may need to reconsider what to do after this step
                // maybe if there are no balls left, turn off indexer and return, otherwise start a new generalized release routine
                new InstantCommand (() -> {
                    indexer.progressBalls();
                    if (indexer.hasOneBall()) {
                        indexer.indexerMotor.set(Constants.Indexer.kIndexerOff);
                    }
                })
            );
        }
    }
}
