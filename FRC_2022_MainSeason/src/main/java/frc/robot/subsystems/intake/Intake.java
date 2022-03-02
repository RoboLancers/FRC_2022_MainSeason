package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake extends SubsystemBase {
    public CANSparkMax intakeMotor; // for roller
    public DoubleSolenoid retractionPiston; // for pistons
    public boolean intakeOn = false;

    public Intake() {
        this.intakeMotor = new CANSparkMax(Constants.Intake.kIndexerPort, MotorType.kBrushless); //intake motor for roller
        intakeMotor.setIdleMode(IdleMode.kCoast); // This means that the motor running the intake will not brake
        this.retractionPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Intake.kPistonDeploy, Constants.Intake.kPistonRetract);
        retractionPiston.set(Value.kReverse);
    }

    public void toggleDeploy() {
        retractionPiston.toggle();
    }

    public void toggleIntake() {
        if (intakeOn) {
            intakeMotor.set(0);
            intakeOn = false;
        }
        else {
            intakeMotor.set(Constants.Intake.kIntakePower);
        }
    }

    public Value isIntakeDown() {
        return (retractionPiston.get());
    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }
}