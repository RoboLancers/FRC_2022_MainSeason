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
        
        this.addRequirements(this.turret);
    }

    @Override
    public void execute(){
        this.turret.pitch.PIDController.setReference(12, CANSparkMax.ControlType.kPosition);

        this.turret.flywheel.PIDControllerA.setReference(this.turret.flywheel.velocitySetpoint, CANSparkMax.ControlType.kVelocity);
        this.turret.flywheel.PIDControllerB.setReference(this.turret.flywheel.velocitySetpoint, CANSparkMax.ControlType.kVelocity);
        if(this.turret.flywheel.velocitySetpoint == 0){
            this.turret.flywheel.motorA.set(0);
            this.turret.flywheel.motorB.set(0);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
