import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;

SlewRateLimiter throttleFilter = new SlewRateLimiter(0.5);
SlewRateLimiter turnFilter = new SlewRateLimiter(0.5);



public class UseDrivetrain extends CommandBase {
    public final Drivetrain drivetrain;
    public final XboxController xboxcontroller = new XboxController(1);

    public UseDrivetrain(Drivetrain drivetrain, XboxController xboxController) {
        this.drivetrain = drivetrain;
        this.xboxController = xboxController;
        
    }
    @Override 
    public void execute () {
        
        maxPower = 0.75;
        
        throttle = throttleFilter.caclulate(xboxController.getY(kLeft));

        turn = turnFilter.calculate.(xboxController.getX(kRight));

        Drivetrain.arcadeDrive(throttle, turn);

        }
        
    @Override
    public boolean isFinished(){
        return false;    }
}