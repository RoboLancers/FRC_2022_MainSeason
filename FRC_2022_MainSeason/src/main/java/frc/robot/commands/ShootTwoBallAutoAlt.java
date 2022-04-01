package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.LowHubShot;
//import frc.robot.subsystems.turret.commands.ActiveLaunchTrajectory;
import frc.robot.subsystems.turret.commands.UpperHubShot;
import frc.robot.subsystems.turret.commands.ZeroPitch;
//import frc.robot.subsystems.drivetrain.command.ToggleGearShifter; is used to switch from slower to faster speed and vice versa (gear shifter covers motor, weird metal cylinder i believe)
import frc.robot.subsystems.intake.Intake;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.Constants; 
 
 
 
public class ShootTwoBallAutoAlt extends SequentialCommandGroup {
    public ShootTwoBallAutoAlt (Drivetrain drivetrain, Turret turret, Indexer indexer, Intake intake) {
       
        addRequirements(turret,indexer, drivetrain, intake);
 
        addCommands(
            new ZeroPitch(turret),
           
            new ParallelRaceGroup(new UpperHubShot(turret), new SequentialCommandGroup(new WaitCommand(1.5),
            new InstantCommand(()-> indexer.setPower(1.0)), new WaitCommand(1.5)
           ), 
           new PrintCommand ("beggining"), new WaitCommand(1.5), new PrintCommand ("end"),
           new InstantCommand(()-> indexer.setPower(0.0))),
 
           //index and shoot ball
         new InstantCommand(()->intake.setPower(0.8)),
        
         new ParallelRaceGroup( new RunCommand(()-> drivetrain.arcadeDrive(0.4, 0)), new WaitCommand(2)),
        new InstantCommand(()-> drivetrain.arcadeDrive(0.4, 0.0)),
       
        //new UseIntake(intake),
       
        new ParallelRaceGroup( new RunCommand(()-> drivetrain.arcadeDrive(-0.4,0)), new WaitCommand(2.5)),
        new ParallelRaceGroup( new RunCommand(()-> drivetrain.arcadeDrive(0,0)), new WaitCommand(0)),

        new ParallelRaceGroup(
            new LowHubShot(turret), 
            new SequentialCommandGroup(
                new WaitCommand(1.5),
                new InstantCommand(()-> indexer.setPower(1.0)),
                new WaitCommand(1.5)
           )
          ),
          new InstantCommand(()-> indexer.setPower(0.0)), new InstantCommand(()->intake.setPower(0.0)));
 
       
       
 
        //shoot, wait, taxi
        //Parallel Race Group: Runs multiple commands at the same time
        //Wait Command: waits (lol) in seconds
        //Run Command: runs the command that's given
        //Sequential Command 

        //Take One: Notes

        //Intake needs to run for a longer period of time
        //Look at othe two ball auto for shooting syntax
        //Did not move away far enough from tarmax
        
        //when I stop indexer, stop intake immediately after 
 
    }
}

