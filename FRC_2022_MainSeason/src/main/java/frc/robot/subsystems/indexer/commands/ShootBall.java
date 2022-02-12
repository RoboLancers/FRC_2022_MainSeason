package frc.robot.subsystems.indexer.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;

public class ShootBall extends CommandBase {
    public Indexer indexer;
    public double initialFieldTime;
    public double currentFieldTime;
    public int secondBallPosition;

    public ShootBall(Indexer indexer) {
        this.indexer = indexer;
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
    public void end() {
        if (indexer.balls[1] == null) {
            if (DriverStation.getMatchTime() >= initialFieldTime + Constants.Indexer.kShootTime) {
                indexer.indexerMotor.set(Constants.Indexer.kStandardIndexerSpeed);
                indexer.balls[0] = indexer.balls[1];
                indexer.balls[1] = null;
            }
        }
        else if (indexer.balls[1] != null) {
            if (balls[1].getPos = balls[1].getPos) {
                indexer.indexerMotor.set(Constants.Indexer.kIndexerOff);
            }
        }
    }
    
}

// Current based, not a while loop