package frc.robot.subsystems.intake.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class UseIntake extends CommandBase{
    double intakeMotorPower;
    double rollerMotorPower;
    public UseIntake(double rollerMotorPower, double transferMotorPower, Intake intake) {
        this.intakeMotorPower = intakeMotorPower;
        this.rollerMotorPower = rollerMotorPower;
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