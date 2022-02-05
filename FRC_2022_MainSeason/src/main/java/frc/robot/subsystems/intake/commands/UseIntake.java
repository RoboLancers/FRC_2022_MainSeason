package frc.robot.subsystems.intake.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class UseIntake extends CommandBase{
    double intakeMotorPower;
    double transferMotorPower;
    public UseIntake(double rollerMotorPower, double transferMotorPower) {
        this.intakeMotorPower = intakeMotorPower;
        this.transferMotorPower = transferMotorPower;
    }

    @Override 
    public void execute() {

    }   

    @Override
    public boolean isFinished() {
        return false;
    }
}