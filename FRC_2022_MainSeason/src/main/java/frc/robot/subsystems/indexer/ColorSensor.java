package frc.robot.subsystems.indexer;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.awt.Color;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.indexer.BallColor;

public class ColorSensor extends SubsystemBase {
    
    // Maintains the sensor closest to the intake
    public final ColorSensorV3 firstColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    // Maintains the sensor closest to the turret
    public final ColorSensorV3 secondColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    public BallColor ballColor = new BallColor();

    
    // Maintains the color of the first ball
    public Color firstSensorBallColor;

    // Maintains the color of the second ball
    public Color secondSensorBallColor;

    // Maintains the number of balls in the indexer
    public int numberOfBalls;

    // Checks if this is the first time the ball has been detected by the first sensor
    public boolean firstDetected;

    // Checks if this is the first time the ball has been detected by the second sensor
    public boolean secondDetected;

    // Maintains the color of the first ball, if present
    public Color firstBallColor;

    // Maintains the color of the second ball, if present
    public Color secondBallColor;

    // Maintains the time when the second sensor was triggered
    public double timeOfTriggering;

    //
    public boolean ballHasBeenShot;

    // Finds the color of the first ball
    public void currentFirstBallColor() {
        firstSensorBallColor = ballColor.getColor(firstColorSensor.getBlue(), firstColorSensor.getRed());
        if (firstSensorBallColor == Color.white) {
            firstDetected = true;
        }
        else if (firstDetected) {
            firstDetected = false;
            numberOfBalls++;
            if ((firstSensorBallColor == Color.blue) && (numberOfBalls == 0)) {
                firstBallColor = Color.blue;
            }
            else if ((firstSensorBallColor == Color.red) && (numberOfBalls == 0)) {
                firstBallColor = Color.red;
            }
            else if ((firstSensorBallColor == Color.blue) && (numberOfBalls == 1)) {
                secondBallColor = Color.blue;
            }
            else if ((firstSensorBallColor == Color.red) && (numberOfBalls == 1)) {
                secondBallColor = Color.red;
            }
        }
    }

    // Finds the color of the second ball
    public void currentSecondBallColor() {
        secondSensorBallColor = ballColor.getColor(secondColorSensor.getBlue(), secondColorSensor.getRed());
        if (secondSensorBallColor == Color.white) {
            secondDetected = true;
        }
        else if (secondDetected) {
            secondDetected = false;
            timeOfTriggering = Timer.getMatchTime()
            ;
        }
    }

    // Stores the colors of the two balls dependent on if they have been shot
    public void currentBalls() {
        if (ballHasBeenShot) { // Ask shooter team to create a has been shot method
            numberOfBalls--;
            firstBallColor = secondBallColor;
            secondBallColor = Color.white;
        }
        if (numberOfBalls == 0) {
            firstBallColor = Color.white;
            secondBallColor = Color.white;
        }
    }

    public void outputToDashBoard() {
        if (numberOfBalls == 1) {
            SmartDashboard.putString("First Ball Color", firstBallColor.toString());
            SmartDashboard.putNumber("Number of Balls", numberOfBalls);
        }
        else if (numberOfBalls == 2) {
            SmartDashboard.putString("First Sensor Color", firstBallColor.toString());
            SmartDashboard.putString("Second Ball Color", secondBallColor.toString());
            SmartDashboard.putNumber("Number of Balls", numberOfBalls);
        }
    }

    // Updates the color values onto the smart dashboard
    public void periodic() {
        SmartDashboard.updateValues();
    }
}
