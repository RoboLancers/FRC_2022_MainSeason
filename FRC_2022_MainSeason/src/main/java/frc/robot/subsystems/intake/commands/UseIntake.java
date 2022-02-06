package frc.robot.subsystems.intake.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class UseIntake extends CommandBase{
    double intakeMotorPower;
    double transferMotorPower;
    public UseIntake(double rollerMotorPower, double transferMotorPower) { // You'll need to add the intake subsystem to the contsructor
        this.intakeMotorPower = intakeMotorPower;
        this.transferMotorPower = transferMotorPower; // Transfer motor should not be used in this class
    }

    @Override
    public void execute() {
        // Finn: Just put a constant in constants.java called kIntakePower, then do intake.setPower(kIntakePower);
    }   

    @Override
    public boolean isFinished() {
        return false;
    }

    // You need to override the end() method, which should just set the intake power to 0
}