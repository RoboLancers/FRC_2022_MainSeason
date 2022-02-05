package frc.robot.subsystems.lights.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.lights.AddressableLEDs;
import frc.robot.subsystems.lights.enums.Colors;


public class SetLoading extends CommandBase {

    AddressableLEDs m_AddressableLEDs;
    public SetLoading(int numPixels) {
        m_AddressableLEDs = new AddressableLEDs(numPixels);
    }

    @Override
    public void initialize() { 
    }

    @Override
    public void execute() {
        m_AddressableLEDs.setIndexerColor(Colors.YELLOW);
    }
}