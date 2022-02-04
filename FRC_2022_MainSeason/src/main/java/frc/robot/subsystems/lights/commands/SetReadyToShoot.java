package frc.robot.subsystems.lights.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.lights.AddressableLEDs;
import frc.robot.subsystems.lights.enums.Colors;

public class SetReadyToShoot extends CommandBase {

    @Override
    public void initialize() {
        AddressableLEDs m_AddressableLEDs = new AddressableLEDs(); 
    }

    @Override
    public void execute() {
        m_AddressableLEDs.setIndexerColor(Colors.GREEN);
    }

}