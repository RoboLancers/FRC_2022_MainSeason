package frc.robot.subsystems.lights;

import frc.robot.sybsystems.lights.enums.ValuesToColors.Colors;


public class AddressableLEDs extends SubsystemBase {

    @Override
    public AddressableLEDs(int numPixels) { //create a constructor, not an initialization
        AddressableLED m_led = new AddressableLED(0);
        
        AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(numPixels);
        m_led.setLength(m_ledBuffer.getLength()) //argument is number of LEDs
        setReadyToShoot();
    }

    //use this function for all other led functions
    public setIndexerColor(Colors color1, Colors color2) {
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
    public setIndexerColor(Colors color1) {
 
        for (int i = 1; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, color1.red, color1.green, color1.blue);
        } 
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
 

    //have a command group such that if cargo is loaded into turret, run this method
    public setReadyToShoot() {
        m_led.setRGB(); //set green
        
        for (int i = 1; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, GREEN.red, GREEN.green, GREEN.blue);
        } 
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    //have a command group such that if hub is out of range, run this method
    public setOutOfRange() {
        m_led.setRGB(); //set red

        for (int i = 1; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, RED.red, RED.green, RED.blue);
        } 
        m_led.setData(m_ledBuffer);
        m_led.start();

    }
    
    public setLoading() {

        for (int i = 1; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, YELLOW.red, YELLOW.green, YELLOW.blue);
        } 
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

}