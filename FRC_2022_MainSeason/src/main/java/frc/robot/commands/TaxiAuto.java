package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class TaxiAuto extends CommandBase{
    Drivetrain drive;

    public TaxiAuto(Drivetrain drive) {
        this.drive = drive;
    }    

    @Override  
    public void execute() {
        drive.curvatureDrive(-0.3, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        drive.tankDriveVolts(0,0);
    }
}
