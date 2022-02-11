package frc.robot.subsystems.intake.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class UseIntake extends CommandBase {
    Intake intake;
    public UseIntake(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.intakeMotor.set(Constants.Intake.kIntakePower);
    }   

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {
        intakeMotorPower = 0;
    }
}