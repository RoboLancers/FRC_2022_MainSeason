package frc.robot.subsystems.misc;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AddressableLEDs extends SubsystemBase {

    PWMSparkMax m_blinkin;

    public AddressableLEDs() {
        this.m_blinkin = new PWMSparkMax(Constants.AddressableLEDs.BlinkinPort);
    }

    public void setLightsColor(double color1) {
        m_blinkin.set(color1);
    }
 
    public void setLoading() {
        setLightsColor(Constants.AddressableLEDs.YELLOW);
    }

    public void setReadyToShoot() {
        setLightsColor(Constants.AddressableLEDs.GREEN);
    }

    public void setMulticolor() {
        setLightsColor(Constants.AddressableLEDs.RED_BLUE);
    }
}