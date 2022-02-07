/*package frc.robot.subsystems.lights.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.lights.AddressableLEDs;
import frc.robot.subsystems.lights.enums.Colors;
import frc.robot.subsystems.lights.ColorSensors;

public class SeekColors() extends CommandBase {
    private ColorSensors m_ColorSensors;
    private AddressableLEDs m_AddressableLEDs;

    public SeekColors() {
        m_ColorSensors = new ColorSensors()
        m_AddressableLEDs = new AddressableLEDs();
    }

    @Override
    public execute() {
        String lowColor = m_ColorSensors.colorMatch(lowColorSensor);
        String highColor = m_ColorSensors.colorMatch(highColorSensor);


        if(lowColor == "blue") {
            if (highColor == "red") {
                m_AddressableLEDs.setIndexerColor(Colors.BLUE, Colors.RED);
            } else {
                m_AddressableLEDs.setIndexerColor(Colors.BLUE);
            }
        }
        
        if(lowColor == "red") {
            if (highColor == "blue") {
                m_AddressableLEDs.setIndexerColor(Colors.RED, Colors.BLUE);
            } else {
                m_AddressableLEDs.setIndexerColor(Colors.RED);
            }
        }
    }
}
*/