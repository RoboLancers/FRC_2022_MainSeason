package frc.robot.subsystems.intake.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;


public class UseIntake extends CommandBase{
    public Intake intake;

    public UseIntake(Intake intake) {
        this.intake = new Intake();
    }

    @Override
    public void execute() {
        intake.toggle(); //extends pistons
    }   

    @Override
    public boolean isFinished() {
        return false;
    }
}