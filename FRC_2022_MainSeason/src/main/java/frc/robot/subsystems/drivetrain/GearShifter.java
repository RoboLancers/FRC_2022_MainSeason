package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.enums.GearShifterState;

public class GearShifter extends SubsystemBase {
    private DoubleSolenoid gearShifter;
    private GearShifterState state;

        public GearShifter() {
            gearShifter = new DoubleSolenoid(GEAR_SHIFTER_HIGH, GEAR_SHIFTER_LOW);  // This is the value of the gearshifter as read by the solenoid
            state = GearShifterState.HIGHGEAR; // created variable to represent the current gearshifter state
        }
    
        public void setGearShifter(GearShifterState gearShifterState) {
            gearShifter.set(gearShifterState.getValue());  //This is changing the value set on the solenoid 
            state = gearShifterState; //this is changing the state of the gear shifter to high gear to low gear or low to high gear
        }
 
        public GearShifterState getState() {
            return state;
        }

        /*public void ToggleGearShifter() {
            gearShifter.setGearShifter(gearShifter.getState() == GearShifterState.HIGHGEAR ? GearShifterState.LOWGEAR : GearShifterState.HIGHGEAR);
        }*/

    
    
}
