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
    public CANSparkMax intakeMotor; // for roller
    public DoubleSolenoid retractionPiston; // for pistons

    public Intake() {
        this.intakeMotor = new CANSparkMax(Constants.Intake.kIndexerPort, MotorType.kBrushless); //intake motor for roller
        intakeMotor.setIdleMode(IdleMode.kCoast); // This means that the motor running the intake will not brake
        this.retractionPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Intake.kPistonDeploy, Constants.Intake.kPistonRetract);
        retractionPiston.set(Value.kReverse);
    }

    public void toggle() {
        retractionPiston.toggle();
    }

    public void setPower() {
        intakeMotor.set(Constants.Intake.kIntakePower);
    }

}
