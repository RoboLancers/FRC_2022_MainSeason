package frc.robot.subsystems.turret.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;

public class ActiveLaunchTrajectory extends CommandBase {
    private Turret turret;

    public ActiveLaunchTrajectory(Turret turret){
        this.turret = turret;
        this.turret.launchTrajectory = new LaunchTrajectory(0, 0);

        SmartDashboard.putBoolean("Zeroed Pitch", false);
        
        this.addRequirements(this.turret);
    }

    @Override
    public void execute(){
        this.turret.pitch.PIDController.setReference(SmartDashboard.getNumber("Target Pitch", 0), CANSparkMax.ControlType.kPosition);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
