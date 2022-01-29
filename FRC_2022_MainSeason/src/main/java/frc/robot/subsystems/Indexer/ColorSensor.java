package frc.robot.subsystems.Indexer;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.awt.Color;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ColorSensor {
    
    // Maintains the sensor closest to the intake
    public final ColorSensorV3 firstColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    // Maintains the sensor closest to the turret
    public final ColorSensorV3 secondColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    // Maintains the color of the first ball
    public Color firstSensorBallColor;

    // Maintains the color of the second ball
    public Color secondSensorBallColor;

    // Finds the color of the first ball
    public void currentFirstBallColor() {
        firstSensorBallColor = getColor(firstColorSensor.getBlue, firstColorSensor.getRed);
        SmartDashboard.putString("First Sensor Color", firstSensorBallColor.toString());
    }

    // Finds the color of the second ball
    public void currentSecondBallColor() {
        secondSensorBallColor = getColor(secondColorSensor.getBlue, secondColorSensor.getRed);
        SmartDashboard.putString("Second Sensor Color", secondSensorBallColor.toString());
    }

    // Updates the color values onto the smart dashboard
    public void periodic() {
        SmartDashboard.updateValues();
    }
}
