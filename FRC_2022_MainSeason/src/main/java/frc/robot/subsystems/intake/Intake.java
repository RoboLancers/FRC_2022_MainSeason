package frc.robot.subsytems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
<<<<<<< HEAD

public class Intake extends SubsystemsBase {
    
    // Maintains the motor for the intake
    public final CANSparkMax rollerMotor = new CANSparkMax(0, MotorType.kBrushless);

    // Maintains the motor for the intake
    public final Solenoid rectractionMotor = new Solenoid(0, MotorType.kBrushless);

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
                rectractionPower.set(true);
            }
            if (isRetracted) {
                rectractionPower.set(false);
            }
        }
=======
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake extends SubsystemBase{
    public CANSparkMax rollerMotor, indexerMotor;
    public Solenoid rectractionMotor;

    public Intake() {
        this.rollerMotor = new CANSparkMax(Constants.Intake.ROLLER_PORT, MotorType.kBrushless); //intake motor for roller
        this.indexerMotor = new CANSparkMax(Constants.Intake.TRANSFER_PORT, MotorType.kBrushless); //transfer motor for indexer
        this.rectractionMotor = new Solenoid(Constants.Intake.RETRACTOR_CHANNEL, Constants.Intake.RETRACTOR_CHANNEL);
>>>>>>> d897f04e036e97c1e0e4d0843bf5d83c2ac50813
    }

    public CANSparkMax getRollerMotor() {return rollerMotor;}

    public CANSparkMax getIndexerMotor() {return indexerMotor;}



}