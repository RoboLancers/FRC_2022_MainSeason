package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

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
    public ColorMatch colorMatcher;

    public Ball(int redValue, int greenValue, int blueValue, BallPosition pos) {
        this.colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(Constants.Indexer.kRedTarget);
        colorMatcher.addColorMatch(Constants.Indexer.kBlueTarget);
        colorMatcher.setConfidenceThreshold(0.75);
        Color ballColor = new Color(redValue, greenValue, blueValue);
        ColorMatchResult colorResult = colorMatcher.matchColor(ballColor);
        if (colorResult.equals(Constants.Indexer.kRedTarget)) {
            color = Color.kRed;
        }
        else if (colorResult.equals(Constants.Indexer.kBlueTarget)) {
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

    public void progressPos(){
        switch(this.pos){
            case BOTTOM:
                this.pos = BallPosition.MIDDLE;
                break;
            case MIDDLE:
                this.pos = BallPosition.TOP;
                break;
            default:
                break;
        }
            
    }
}