package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class UseDrivetrain extends CommandBase {
    public final Drivetrain drivetrain;
    private SlewRateLimiter throttleFilter = new SlewRateLimiter(1.5);
    private SlewRateLimiter turnFilter = new SlewRateLimiter(1.5);
    public XboxController xboxController = new XboxController(1);
    private double maxPower;
    public double throttle;
    public double turn;

    public UseDrivetrain(Drivetrain drivetrain, XboxController xboxController, double maxPower, double throttle, double turn, SlewRateLimiter throttleFilter, SlewRateLimiter turnFilter) {
        this.drivetrain = drivetrain;
        this.xboxController = xboxController;
        this.maxPower = maxPower;
        this.throttle = throttle;
        this.turn = turn;
        this.throttleFilter = throttleFilter;
        this.turnFilter = turnFilter;
    }
    
    @Override 
    public void execute () {
        maxPower = 0.75;
        
        throttle = throttleFilter.calculate(xboxController.getLeftY());

        turn = turnFilter.calculate(xboxController.getRightX());

        drivetrain.curvatureDrive(throttle, turn);
    }
        
    @Override
    public boolean isFinished(){
        return false;
    }
}