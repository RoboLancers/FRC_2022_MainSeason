package frc.robot.subsystems.lights;

/*import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensors() {

    public final ColorSensorV3 lowColorSensor;
    public final ColorSensorV3 highColorSensor;
    public final ColorMatch colorMatcher;

    public ColorSensors() {
        lowColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        highColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        colorMatcher = new ColorMatch();
        colorMatcher.setConfidenceThreshold(0.75);

        colorMatcher.addColorMatch(Constants.Lights.ColorSensor.kBlueTarget);
        colorMatcher.addColorMatch(Constants.Lights.ColorSensor.kRedTarget);
    }

    public string colorMatch(ColorSensorV3 colorSensor) {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorSensor.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
            colorString = "blue";
        } else if (match.color == kRedTarget) {
            colorString = "red";
        } else {
            colorString = "Unknown";
        }

    }

}
*/