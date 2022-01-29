package frc.robot.subsystems.Indexer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.Indexer.ColorSensor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    
    // Maintains the motor for the indexer
    public final CANSparkMax indexerMotor = new CANSparkMax(0, MotorType.kBrushless);

    // Maitains the distance to travel divided by the time
    public final int REQUIRED_TIME;

    public void setIndexerPower() {
        if (numberOfBalls != 2) { // find how to access number of balls
            indexerMotor.setPower(50);
        }
        else if (ColorSensor.timeOfTriggering + REQUIRED_TIME <= Timer.getMatchTime()) {  // find how to access number of balls
            indexerMotor.setPower(0); 
        }
        else {
            indexerMotor.setPower(50);
        }
    }

}
