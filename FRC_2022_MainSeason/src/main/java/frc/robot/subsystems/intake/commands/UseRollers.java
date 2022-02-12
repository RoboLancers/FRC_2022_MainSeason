package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.inake.Intake;

public class UseRollers extends CommandBase {
    public Intake intake;

    public UseRollers(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void toggle() {
        if (intake.intakeMotor.get() >= Constants.Intake.kIntakePower - Constants.Intake.kErrorMargin) {
            intake.intakeMotor.set(Constants.Intake.kIntakeOff);
        }
        else {
            intake.intakeMotor.set(Constants.Intake.kIntakePower);
        }
    }
}