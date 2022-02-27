package frc.robot.subsystems.indexer;


import java.util.Queue;
import java.util.PriorityQueue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Ball.BallPosition;
import frc.robot.subsystems.indexer.commands.DefaultIndex;

public class Indexer extends SubsystemBase {
    /*
        Finn: I would suggest instead doing something like this to track the shots.
        - Make a new class called Ball that has a color and a position as shown below
    */
    public final ColorSensorV3 bottomColorSensor = new ColorSensorV3(I2C.Port.kMXP);

    // Maintains the touch sensor closest to the turret
    DigitalInput topLimitSwitch = new DigitalInput(0);
    DigitalInput bottomLimitSwitch = new DigitalInput(1);
    
    public Queue<Ball> ballQueue= new PriorityQueue<Ball>();
    public CANSparkMax indexerMotor = new CANSparkMax(Constants.Indexer.kIndexerPort, MotorType.kBrushless);

    public Indexer(){
        // trigger is a class that'll let you run a command when the trigger is activated
        Trigger threshColorSensor = new Trigger(() -> {
            // the condition that triggers the command
            return bottomColorSensor.getProximity() >= Constants.Indexer.kProximityLimit;
        });
        // This is an example of command composition.
        threshColorSensor.whenActive(new RunCommand(this::processBall) // This runs the processBall() function once the color sensor is activated
            .withInterrupt(this::indexFinished) // Stop this command when the highest ball in the indexer reaches the next color sensor up
            .andThen(() -> indexerMotor.set(0) // After the command is stopped (i.e. the ball reaches the next sensor) stop the indexer motor
        ));

        initDefaultCommand();
    }

    private void initDefaultCommand(){
        setDefaultCommand(new DefaultIndex(this));
    }

    public void processBall() {
        if (ballQueue.size() == 0) { // If no balls are in the indexer
            Ball newBall = new Ball((int) (bottomColorSensor.getColor().red), (int) (bottomColorSensor.getColor().green), (int) (bottomColorSensor.getColor().blue), BallPosition.BOTTOM);
            ballQueue.add(newBall); // Instantiate a new ball in the bottom position
            indexerMotor.set(Constants.Indexer.kIndexerSpeed); // Set the indexer to the speed we want
            newBall.setPos(BallPosition.MIDDLE); // Check to see if this works
        } else { // If one ball is indexed, and another is detected, we want to move the indexed one to the top position, which should bring the bottom one into the indexer
            ballQueue.add(new Ball(bottomColorSensor.getRed(), bottomColorSensor.getGreen(), bottomColorSensor.getBlue(), BallPosition.BOTTOM)); // Instantiate a new ball in the bottom
            indexerMotor.set(Constants.Indexer.kIndexerSpeed);
        }
    }

    public boolean indexFinished() {
        if (ballQueue.peek().getPos() == BallPosition.MIDDLE && topLimitSwitch.get()) { // if the indexer is running until a ball reaches the top
            ((Ball[]) ballQueue.toArray())[1].setPos(BallPosition.MIDDLE);; // move both balls up
            ballQueue.peek().setPos(BallPosition.TOP);
            return true;
        } else if (ballQueue.peek().getPos() == BallPosition.BOTTOM && bottomLimitSwitch.get()) { // if the indexer is running until a ball reaches the middle
            ballQueue.peek().setPos(BallPosition.MIDDLE); // move the ball up
            return true;
        }
        return false;
    }

    public boolean inShootingPosition() {
        return (ballQueue.size() > 0);
    }

    public boolean hasTwoBalls() {
        return (ballQueue.size() == 2);
    }

    public boolean hasOneBall() {
        return (ballQueue.size() == 1);
    }

    public boolean hasManyBalls() {
        return (ballQueue.size() >= 3);
    }

    public boolean hasFewBalls() {
        return (ballQueue.size() < 3);
    }

    public void progressBalls() {
        if (ballQueue.size() == 1) {
            if (ballQueue.peek().getPos() == BallPosition.TOP) {
                ballQueue.poll();
            }
            ballQueue.peek().progressPos();
        }
        else if (ballQueue.size() == 2) {
            Ball[] ballArray = ((Ball[]) ballQueue.toArray());
            ballArray[0].progressPos();
            ballArray[1].progressPos();
        }
        else if (ballQueue.size() > 2) {
            Ball[] ballArray = ((Ball[]) ballQueue.toArray());
            for (int i = 0; i < ballArray.length; i++) {
                ballArray[i].progressPos();
            }
        }
    }

    public void setPower(double power) {
        indexerMotor.set(power);
    }
}