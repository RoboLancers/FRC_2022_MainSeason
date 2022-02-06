package frc.robot.subsystems.indexer;

import java.awt.Color;

import frc.robot.Constants;


public class BallColor {
    
    // Maintains the minimum red value of the ball to be considered red
    
    // Maintains the minimum blue value of the ball to be considered blue

    // Returns the color of the ball based on the given RGB values
    public Color getColor(int blueValue, int redValue) {
        if ((redValue >= Constants.Intake.kRedThreshold) && !(blueValue >= Constants.Intake.kBlueThreshold)) {
            return Color.red;
        }
        else if ((blueValue >= Constants.Intake.kBlueThreshold) && !(redValue >= Constants.Intake.kRedThreshold)) {
            return Color.blue;
        } // Finn: does it make sense to add another else if branch for if they're both above the threshold, taking the larger one?
        else {
            return Color.white;
        }
    }
}
