package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;
import edu.wpi.first.wpilibj.DriverStation;

public class ShootBall extends CommandBase {
    public Indexer indexer;
    public double initialFieldTime;
    public double currentFieldTime;
    public int secondBallPosition;

    public ShootBall(Indexer indexer) {
        this.indexer = indexer;

        addRequirements(indexer);
    }

    public void execute() {
        if (indexer.balls[1] == null) {
            indexer.indexerMotor.set(Constants.Indexer.kIndexerSpeed);
            initialFieldTime = DriverStation.getMatchTime();
        }
        else if (indexer.balls[1] != null) {
            indexer.indexerMotor.set(Constants.Indexer.kIndexerSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (indexer.balls[1] == null) {
            if (DriverStation.getMatchTime() >= initialFieldTime + Constants.Indexer.kShootTime) {
                indexer.indexerMotor.set(Constants.Indexer.kStandardIndexerSpeed);
                indexer.balls[0] = indexer.balls[1];
                indexer.balls[1] = null;
            }
        }
        else if (indexer.balls[1] != null) {
            if (indexer.balls[1].getPos() == indexer.balls[1].getPos()) {
                indexer.indexerMotor.set(Constants.Indexer.kIndexerOff);
            }
        }
    }
    
}

// Current based, not a while loop
// Potential problem, it only checks fi the first has passed the final trigger, the first might no have been shot