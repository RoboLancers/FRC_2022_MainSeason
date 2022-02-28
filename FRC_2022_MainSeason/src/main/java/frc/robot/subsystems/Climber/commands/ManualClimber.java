package frc.robot.subsystems.climber.commands;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.XboxController;

public class ManualClimber extends CommandBase {
    private XboxController climberController;
    private final CANSparkMax climberMotor1 = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax climberMotor2 = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
    public ManualClimber(ManualClimber ManualClimber, XboxController climberController){
        this.climberController = climberController;
    }

    @Override
    public void execute(){
        climberController.getAxisValue(XboxController.Axis.LEFT_Y);

}
}