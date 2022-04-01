package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;


public class UseIntake extends CommandBase{ // DeployIntake
    public Intake intake;

    public UseIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.toggleDeploy();; //extends pistons
    }   

    @Override
    public boolean isFinished() {
        return false;
    }
}