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
 
    public void setHanging() {
        setLightsColor(Constants.AddressableLEDs.CRIMSON_GOLD);
    }

    public void setNoCargo() {
        setLightsColor(Constants.AddressableLEDs.WHITE);
    }

    public void setOneCargo() {
        setLightsColor(Constants.AddressableLEDs.YELLOW);
    }

    public void setTwoCargo() {

    }

    public void setDistanceBad() {
        setLightsColor(Constants.AddressableLEDs.RED);
    }

    public void setFiringUp() {
        setLightsColor(Constants.AddressableLEDs.BLUE);
    }

    public void setTargetLocked() {
        setLightsColor(Constants.AddressableLEDs.GREEN);
    }
}