package frc.robot.subsystems.climber.commands;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.XboxController;

public class ManualClimber extends CommandBase {
    private final XboxController ClimbController;
    private Climber climber;
    
    public ManualClimber(XboxController ClimbController, Climber climber){
        this.climber = climber;
        this.ClimbController = ClimbController;
        addRequirements(climber);
    }
    

    public void execute(){
        climber.climberMotor1.set(ClimbController.getAxisValue(XboxController.Axis.LEFT_Y));
        climber.climberMotor2.set(ClimbController.getAxisValue(XboxController.Axis.RIGHT_Y));
    }
}