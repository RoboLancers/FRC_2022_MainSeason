package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    private Compressor compressor;

    public Pneumatics(){
        compressor = new Compressor(PneumaticsModuleType.REVPH);
    }

    /**
     * Regulates the compressor
     */
    public void regulateCompressor(){
        if(!compressor.getPressureSwitchValue() && !compressor.enabled()
                && isCompressorSafeToUse()){
            compressor.enableDigital();
        }else if(compressor.getPressureSwitchValue() && compressor.enabled()
                || !isCompressorSafeToUse()){
            compressor.disable();
        }
    }

    /**
     * Checks if compressor is safe to use
     * @return whether or not the compressor is safe to use
     */
    private boolean isCompressorSafeToUse(){
        return true;
    }

    public void stopCompressor(){
        compressor.disable();
    }
}