package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Wiggle extends CommandBase {
    private Drivetrain drivetrain;
    private double x;

    
    public Wiggle(Drivetrain drivetrain){
        SmartDashboard.putNumber("dX", SmartDashboard.getNumber("dX", 0.27));
        this.drivetrain = drivetrain;

        addRequirements(this.drivetrain);
    }

    @Override
    public void execute(){
        this.x += SmartDashboard.getNumber("dX", 0.27);
        this.drivetrain.arcadeDrive(0, 0.3 * Math.sin(x), false);
    }

    @Override
    public void end(boolean interrupted){
        this.drivetrain.arcadeDrive(0, 0, false);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
