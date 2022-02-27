package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;

public class DefaultIndex extends CommandBase {
    private Indexer indexer;

    public DefaultIndex(Indexer indexer){
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute(){
        this.indexer.indexerMotor.set(Constants.Indexer.kIndexerOff);
        
        SmartDashboard.putNumber("proximity", indexer.bottomColorSensor.getProximity());
        SmartDashboard.putNumber("red", indexer.bottomColorSensor.getRed());
        SmartDashboard.putNumber("blue", indexer.bottomColorSensor.getBlue());
        SmartDashboard.putNumber("green", indexer.bottomColorSensor.getGreen());
        SmartDashboard.putNumber("ball number", indexer.ballQueue.size());
    }
}
