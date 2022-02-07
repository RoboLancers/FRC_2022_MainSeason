package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class Ball { // Finn: This class is just a simple utility to hold some information about the ball's indexing state
    public enum BallPosition {
        BOTTOM (0), MIDDLE (1), TOP(2);

        int posNumber;
        BallPosition(int posNumber) {
            this.posNumber = posNumber;
        }
    
    }
    private final Color color;
    private BallPosition pos;

    public Ball(int redValue, int blueValue, BallPosition pos) {

        if ((redValue >= Constants.Intake.kRedThreshold) && !(blueValue >= Constants.Intake.kBlueThreshold)) {
            color = Color.kRed;
        }
        else if ((blueValue >= Constants.Intake.kBlueThreshold) && !(redValue >= Constants.Intake.kRedThreshold)) {
            color = Color.kBlue;
        } 
        else {
            color = Color.kWhite;
        }
    }

    public Color getColor() {
        return color;
    }

    public BallPosition getPos() {
        return pos;
    }

    public void setPos(BallPosition pos) {
        this.pos = pos;
    }

    public void progressPos() {
        pos.posNumber++; // does this work? Check with Matt
    }
}
