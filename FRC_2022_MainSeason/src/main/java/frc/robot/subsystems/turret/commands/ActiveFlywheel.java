package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;

public class ActiveFlywheel extends CommandBase {
    private Turret turret;

    private PIDController pidController;
    private SimpleMotorFeedforward ffController;

    public ActiveFlywheel(Turret turret){
        this.turret = turret;

        this.pidController = new PIDController(
            Constants.Turret.Flywheel.kP,
            Constants.Turret.Flywheel.kI,
            Constants.Turret.Flywheel.kD
        );
        this.pidController.reset();

        this.ffController = new SimpleMotorFeedforward(
            0.18017,
            0.12747,
            0.0038965
        );

        this.addRequirements(this.turret.flywheel);
    }

    @Override
    public void execute(){
        this.turret.flywheel.setVoltage(
            this.pidController.calculate(this.turret.flywheel.getVelocity(), this.turret.launchTrajectory.speed) +
            this.ffController.calculate(this.turret.flywheel.getVelocity(), this.turret.launchTrajectory.speed)
        );
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        this.turret.flywheel.setVoltage(0);
    }
}