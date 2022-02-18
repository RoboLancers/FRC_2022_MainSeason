package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Ball.BallPosition;

public class Indexer extends SubsystemBase {

    /*
    Finn: I would suggest instead doing something like this to track the shots.
     - Make a new class called Ball that has a color and a position as shown below
    */
    public final ColorSensorV3 bottomColorSensor = new ColorSensorV3(I2C.Port.kOnboard); // not sure what this error is, but you should fix it

    // Maintains the touch sensor closest to the turret
    DigitalInput toplimitSwitch = new DigitalInput(0);
    DigitalInput bottomlimitSwitch = new DigitalInput(1);
    
    public Ball[] balls = new Ball[2]; // Create an array of balls
    public CANSparkMax indexerMotor = new CANSparkMax(Constants.Indexer.kIndexerPort, MotorType.kBrushless);

    public Indexer(){
        Trigger threshColorSensor = new Trigger() { // trigger is a class that'll let you run a command when the trigger is activated
        @Override
        public boolean get() { // this is the condition to run our command (i.e. the ball is close enough, run the command)
            return bottomColorSensor.getProximity() >= Constants.Indexer.kProximityLimit;
            }
        };
        // This is an example of command composition.
        threshColorSensor.whenActive((new RunCommand(this::processBall)) // This runs the processBall() function once the color sensor is activated
        .withInterrupt(this::indexFinished) // Stop this command when the highest ball in the indexer reaches the next color sensor up
        .andThen(() -> indexerMotor.set(0) // After the command is stopped (i.e. the ball reaches the next sensor) stop the indexer motor
        ));
    }

    public void processBall() {
        if (balls[0] == null && balls[1] == null) { // If no balls are in the indexer
            balls[0] = new Ball(bottomColorSensor.getRed(), bottomColorSensor.getBlue(), BallPosition.BOTTOM); // Instatiate a new ball in the bottom position
            indexerMotor.set(Constants.Indexer.kIndexerSpeed); // Set the indexer to the speed we want
            balls[0].progressPos();
        } else if (balls[1] == null) { // If one ball is indexed, and another is detected, we want to move the indexed one to the top position, which should bring the bottom one into the indexer
            balls[1] = new Ball(bottomColorSensor.getRed(), bottomColorSensor.getBlue(), BallPosition.BOTTOM); // Instantiate a new ball in the bottom
            indexerMotor.set(Constants.Indexer.kIndexerSpeed);
        } else { // This is an edge case that comes up if one ball is at the top and the other ball is at the bottom, triggering the bottom sensor
            // which would mean that something went wrong with our indexing, and we should bring the top ball back to the middle and try again.
            balls[0].setPos(BallPosition.BOTTOM); // this just makes sure that the next ball to come through the middle is set as the middle ball
            indexerMotor.set(-Constants.Indexer.kIndexerSpeed); // run the indexer down
        }
    }

    public boolean indexFinished() {
        if (balls[0].getPos() == BallPosition.MIDDLE && toplimitSwitch.get()) { // if the indexer is running until a ball reaches the top
            balls[1].setPos(BallPosition.MIDDLE); // move both balls up
            balls[0].setPos(BallPosition.TOP);
            return true;
        } else if (balls[0].getPos() == BallPosition.BOTTOM && toplimitSwitch.get()) { // if the indexer is running until a ball reaches the middle
            balls[0].setPos(BallPosition.MIDDLE); // move the ball up
            return true;
        }
        return false;
    }

    public boolean inShootingPosition() {
        if ((balls[0].getColor() != null) && (balls[1].getColor() != null)) {
            if ((balls[0].getPos() == BallPosition.TOP) && ((balls[0].getPos() == BallPosition.BOTTOM) || (balls[0].getPos() == BallPosition.MIDDLE))) {return true;}
            return false;
        }
        else if (balls[0].getColor() != null) {
            if (balls[0].getPos() == BallPosition.TOP) {return true;}
            return false;
        }
        else {return false;}
    }

    public boolean hasTwoBalls() {
        return (balls[1] != null);
    }
}
