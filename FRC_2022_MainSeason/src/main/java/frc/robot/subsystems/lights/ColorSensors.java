package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C.Port;

public class ColorSensors {

    public final ColorSensorV3 lowColorSensor;
    public final ColorSensorV3 highColorSensor;
    public final ColorMatch colorMatcher;

    public ColorSensors() {
        lowColorSensor = new ColorSensorV3(Port.kOnboard);
        highColorSensor = new ColorSensorV3(Port.kOnboard);
        colorMatcher = new ColorMatch();
        colorMatcher.setConfidenceThreshold(0.75);

        colorMatcher.addColorMatch(Constants.Lights.ColorSensor.kBlueTarget);
        colorMatcher.addColorMatch(Constants.Lights.ColorSensor.kRedTarget);
    }

    /*
    //what is this method supposed to do?
    public String colorMatch(ColorSensorV3 colorSensor) {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorSensor.getColor().matchColor(detectedColor);

        String colorString;
        if (match.color == Constants.Lights.ColorSensor.kBlueTarget) {
            return colorString = "blue";
        } else if (match.color == Constants.Lights.ColorSensor.kRedTarget) {
            return colorString = "red";
        } else {
            return colorString = "Unknown";
        }

    }
    */

}
