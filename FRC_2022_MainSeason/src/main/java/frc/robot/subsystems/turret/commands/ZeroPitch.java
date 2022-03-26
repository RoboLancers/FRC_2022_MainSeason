package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class ZeroPitch extends CommandBase {
    private Turret turret;

    public ZeroPitch(Turret turret){
        this.turret = turret;

        this.turret.pitch.enableSoftLimits(false);

        addRequirements(this.turret);
    };

    @Override
    public void execute(){
        this.turret.pitch.setMotorPower(-0.25);
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            this.turret.pitch.resetEncoder();
        }
        this.turret.pitch.enableSoftLimits(true);
    }

    @Override
    public boolean isFinished(){
        return this.turret.pitch.limitSwitchTriggered();
    }
}