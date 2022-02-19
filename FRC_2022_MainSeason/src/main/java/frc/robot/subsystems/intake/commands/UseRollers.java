package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class UseRollers extends CommandBase {
    public UseRollers(Intake intake) {
        if (intake.intakeMotor.get() >= Constants.Intake.kIntakePower - Constants.Intake.kErrorMargin) {
            intake.intakeMotor.set(0.0);
        }
        else {
            intake.intakeMotor.set(Constants.Intake.kIntakePower);
        }
    }
}