import frc.robot.util.Controller;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class UseDrivetrain extends CommandBase {
    public final Drivetrain drivetrain;
    public final Controller controller = new Controller(1);

    SlewRateLimiter throttleFilter = new SlewRateLimiter(0.5);
    SlewRateLimiter turnFilter = new SlewRateLimiter(0.5);

    public UseDrivetrain(Drivetrain drivetrain, Controller controller) {
        this.drivetrain = drivetrain;
        this.Controller = Controller;
        
    }
    @Override 
    public void execute () {
        maxPower = 0.75;
        
        throttle = Controller.getAxisValue(Controller.Axis.LEFT_Y);

        turn = Controller.getAxisValue(Controller.Axis.LEFT_X);
        
        throttle = (throttle < 0 ? Math.max(-maxPower, throttle) : Math.min(maxPower, throttle));

        Drivetrain.arcadeDrive(throttle, turn);
        
        
        }
        
    @Override
    public boolean isFinished(){
        return false;    }
}