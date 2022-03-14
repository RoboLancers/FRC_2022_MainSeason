package frc.robot.subsystems.turret.commands;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;

public class ZeroPitch extends CommandBase {
    private Turret turret;

    public ZeroPitch(Turret turret){
        this.turret = turret;

        this.turret.pitch.motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        this.turret.pitch.motor.enableSoftLimit(SoftLimitDirection.kForward, false);

        SmartDashboard.putBoolean("Zeroed Pitch", true);

        addRequirements(this.turret);
    };

    @Override
    public void execute(){
        this.turret.pitch.motor.set(-0.1);
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            this.turret.pitch.encoder.setPosition(0);
        }
        this.turret.pitch.motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        this.turret.pitch.motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    @Override
    public boolean isFinished(){
        return !this.turret.pitch.homingSwitch.get();
    }
}