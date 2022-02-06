package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import frc.robot.Constants.Indexer;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Intake extends SubsystemBase{
    public CANSparkMax intakeMotor, indexerMotor;
    public DoubleSolenoid retractionPiston;

    public Intake() {
        this.intakeMotor = new CANSparkMax(Constants.Intake.kIndexerPort, MotorType.kBrushless); //intake motor for roller
        intakeMotor.setIdleMode(IdleMode.kCoast); // This means that the motor running the intake will not brake
        this.retractionPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Intake.kPistonDeploy, Constants.Intake.kPistonRetract);

        // The piston needs additional logic. Something along the lines of:
        // retractionPiston.set(Value.kReverse); 
        // To explain, kReverse just means that the piston will go to its retracted position. kForward is the extended position
        // Double action solenoids can use a toggle() method, but they need to be set to a position initially
        
    }

    public CANSparkMax getIntakeMotor() {return intakeMotor;} // You don't need this method

    // You should make a setPower function that does intakeMotor.set()

    // You should make a toggle() function that calls retractionPiston.toggle()

}