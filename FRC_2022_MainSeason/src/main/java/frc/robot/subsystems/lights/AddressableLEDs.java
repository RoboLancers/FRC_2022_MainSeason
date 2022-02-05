package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lights.enums.Colors;

public class AddressableLEDs extends SubsystemBase {

    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    public AddressableLEDs(int numPixels) {
        this.m_led = new AddressableLED(0);
        
        this.m_ledBuffer = new AddressableLEDBuffer(numPixels);
        m_led.setLength(m_ledBuffer.getLength()); //argument is number of LEDs
    }

    //use this function for all other led functions
    public void setIndexerColor(Colors color1, Colors color2) {
        for (int i = 1; i < m_ledBuffer.getLength()/2; i++){
            m_ledBuffer.setRGB(i, color1.red, color1.green, color1.blue);
        } 

        for (int i = m_ledBuffer.getLength()/2 ; i > m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, color2.red, color2.green, color2.blue);
        }

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    //method overloading for if there's only one color needed
    public void setIndexerColor(Colors color1) {
 
        for (int i = 1; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, color1.red, color1.green, color1.blue);
        } 
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
 

    //have a command group such that if cargo is loaded into turret, run this method
    public void setReadyToShoot() {
        
        for (int i = 1; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, Colors.GREEN.red, Colors.GREEN.green, Colors.GREEN.blue);
        } 
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    //have a command group such that if hub is out of range, run this method
    public void setOutOfRange() {

        for (int i = 1; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, Colors.RED.red, Colors.RED.green, Colors.RED.blue);
        } 
        m_led.setData(m_ledBuffer);
        m_led.start();

    }
    
    public void setLoading() {

        for (int i = 1; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, Colors.YELLOW.red, Colors.YELLOW.green, Colors.YELLOW.blue);
        } 
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

}