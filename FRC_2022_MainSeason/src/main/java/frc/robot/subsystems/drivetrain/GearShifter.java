package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.enums.GearShifterState;


public class GearShifter extends SubsystemBase {
    private DoubleSolenoid gearShifter;

        public GearShifter() {
            gearShifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1,3);  // This is the value of the gearshifter as read by the solenoid
            ///gearShifter.set(Value.kReverse);
         } // created variable to represent the current gearshifter state
    
        public void setGearShifter(GearShifterState gearShifterState) {
            gearShifter.set(gearShifterState.getValue());  //This is changing the value set on the solenoid 
        }
 
        public Value getState() {
            return gearShifter.get();
        }

        public void ToggleGearShifter() {
            if (getState() == Value.kOff) {
                gearShifter.set(Value.kReverse);
            } else {
               gearShifter.toggle();
            }
        }

     

    
    
}
