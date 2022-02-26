package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.enums.GearShifterState;

public class GearShifter extends SubsystemBase {
    private DoubleSolenoid gearShifter;

    public GearShifter(Pneumatics pneumatics) {
        gearShifter = pneumatics.getDoubleSolenoid(1, 0);  // value of the gearShifter as read by the solenoid
        // gearShifter.set(Value.kReverse);
        toggleGearShifter();

        initDefaultCommand();
    }
    
    private void initDefaultCommand(){
        // TODO: set default command for gear shifter if needed
    }

    public void setGearShifter(GearShifterState gearShifterState) {
        gearShifter.set(gearShifterState.getValue());  //This is changing the value set on the solenoid 
    }

    public Value getState() {
        return gearShifter.get();
    }

    public void toggleGearShifter() {
        if (getState() == Value.kOff) {
            gearShifter.set(Value.kReverse);
        } else {
            gearShifter.toggle();
        }
    }
}