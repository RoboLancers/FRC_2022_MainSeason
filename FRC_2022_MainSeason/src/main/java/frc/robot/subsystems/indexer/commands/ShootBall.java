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

    public void shoot() {
        if (indexer.balls[1] == null) {
            indexer.indexerMotor.set(Constants.Indexer.kIndexerSpeed);
            initialFieldTime = DriverStation.getMatchTime();
            currentFieldTime = initialFieldTime;
            while (currentFieldTime != initialFieldTime + Constants.Indexer.kShootTime) {
                currentFieldTime = DriverStation.getMatchTime();
            }
            indexer.indexerMotor.set(Constants.Indexer.kStandardIndexerSpeed);
            indexer.balls[0] = indexer.balls[1];
            indexer.balls[1] = null;
        }
        else if (indexer.balls[1] != null) {
            indexer.indexerMotor.set(Constants.Indexer.kIndexerSpeed);
            secondBallPosition = balls[1].getPos
            while (secondBallPosition != 2) { // check to see if 2 is okay
                secondBallPosition = balls[1].getPos;
            }
            indexer.indexerMotor.set(Constants.Indexer.kIndexerOff);
        }
    }
    
}