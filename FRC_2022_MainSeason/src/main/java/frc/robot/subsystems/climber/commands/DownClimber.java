// package frc.robot.subsystems.climber.commands;
// import frc.robot.Constants;
// import frc.robot.subsystems.climber.Climber;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class DownClimber extends CommandBase{
//     private double climberDown;
//     private Climber climber;

//     public DownClimber(Climber climber, double climberDown) {
//         this.climber = climber;
//         this.climberDown = climberDown;
//     }

//     @Override
//     public void execute(){
//         if (climber.climberMotor1.getEncoder().getPosition() > (climberDown)){
//             climber.climberMotor1.set(Constants.Climber.kNegativePower);
//         }
//         else{
//             climber.climberMotor1.set(0);
//         }
//         if (climber.climberMotor2.getEncoder().getPosition() > (climberDown)){
//             climber.climberMotor2.set(Constants.Climber.kNegativePower);
//         }
//         else{
//             climber.climberMotor2.set(0);
//         }
//     }

//     @Override
//     public boolean isFinished(){
//         return (climber.climberMotor2.getEncoder().getPosition() <= (climberDown) && climber.climberMotor2.getEncoder().getPosition() <= (climberDown));
//     }
    
//     @Override
//     public void end(boolean interrupted){
//         climber.climberMotor1.set(0);
//         climber.climberMotor2.set(0);
//     }
// }
