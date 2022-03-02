package frc.robot.subsystems.Climber.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.XboxController;

public class ManualClimber extends CommandBase {
    private XboxController climberController;
    private double manualClimb;

    public ManualClimber(ManualClimber ManualClimber, XboxController climberController){
        this.climberController = climberController;
        this.manualClimb = manualClimb;
    }

    @Override
    public void execute(){
        climberController.getAxisValue(XboxController.Axis.LEFT_Y);

}
}