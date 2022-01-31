package frc.robot.subysystems.intake;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemsBase {
    
    // Maintains the motor for the intake
    public final CANSparkMax rollerMotor = new CANSparkMax(0, MotorType.kBrushless);

    // Maintains the motor for the intake
    public final CANSparkMax rectractionMotor = new CANSparkMax(0, MotorType.kBrushless);

    // Checks to seee if the intake has been retracted
    public boolean isRetracted = false;

    // Sets the power of the rollers
    public void indexerRollerPower() {
        if (retracted) {
            rollerMotor.set(0);
        }
        else {
            rollerMotor.set(50);
        }
    }

    // Retracts the motor depending on if a button was pressed
    public void retractMototor() {
        if (buttonHasBeenPressed) {
            if (isRetracted) {
                rectractionPower.set(50);
            }
            if (isRetracted) {
                rectractionPower.set(0);
            }
        }
    }

}