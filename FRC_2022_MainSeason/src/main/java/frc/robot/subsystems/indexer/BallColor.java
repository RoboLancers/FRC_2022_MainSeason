package frc.robot.subsystems.Indexer;

import java.awt.Color;


public class BallColor {
    
    // Maintains the minimum red value of the ball to be considered red
    public final int RED_MIN = 4000;
    // Maintains the minimum blue value of the ball to be considered blue
    public final int BLUE_MIN = 4000;

    // Returns the color of the ball based on the given RGB values
    public Color getColor(int blueValue, int redValue) {
        if ((redValue >= RED_MIN) && !(blueValue >= BLUE_MIN)) {
            return Color.red;
        }
        else if ((blueValue >= BLUE_MIN) && !(redValue >= RED_MIN)) {
            return Color.blue;
        }
        else {
            return Color.white;
        }
    }
}
