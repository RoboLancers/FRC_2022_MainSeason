package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class UseIntake extends CommandBase{

    public UseIntake(double rollerMotorPower, double transferMotorPower) {
        this.intakeMotorPower = intakeMotorPower;
        this.transferMotorPower = transferMotorPower;

    }

    @Override 
    public void execute() {

    }   

    @Override
    public void end() {
        
    }
}