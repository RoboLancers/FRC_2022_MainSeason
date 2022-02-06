package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class UseDrivetrain extends CommandBase {
    public final Drivetrain drivetrain;
    private SlewRateLimiter throttleFilter = new SlewRateLimiter(Constants.Drivetrain.kThrottleFilter);
    private SlewRateLimiter turnFilter = new SlewRateLimiter(Constants.Drivetrain.kTurnFilter);
    public double throttle;
    public double turn;

    public UseDrivetrain(Drivetrain drivetrain, double throttle, double turn) {
        this.drivetrain = drivetrain;
        this.throttle = throttle;
        this.turn = turn;

    }
    @Override 
    public void execute() {
        
        throttle = throttleFilter.calculate(throttle);

        turn = turnFilter.calculate(turn);

        drivetrain.arcadeDrive(throttle, turn);

        }
        
    @Override
    public boolean isFinished(){
        return false;    
    }
}