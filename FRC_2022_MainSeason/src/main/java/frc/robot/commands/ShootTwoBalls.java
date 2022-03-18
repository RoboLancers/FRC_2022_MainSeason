package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.UseIntake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.ActiveLaunchTrajectory;
import frc.robot.subsystems.turret.commands.UpperHubShoot;
import frc.robot.subsystems.turret.commands.ZeroPitch;
//import frc.robot.subsystems.drivetrain.command.ToggleGearShifter; is used to switch from slower to faster speed and vice versa (gear shifter covers motor, weird metal cylinder i believe)
import edu.wpi.first.wpilibj2.command.WaitCommand;



public class ShootTwoBalls extends SequentialCommandGroup {
    public ShootTwoBalls (Drivetrain drivetrain, Turret turret, Indexer indexer, Intake intake) {
        
        addRequirements(indexer, drivetrain);

        addCommands(
            new ZeroPitch(turret),
           
            new ParallelRaceGroup(new UpperHubShoot(turret), new SequentialCommandGroup(new WaitCommand(1.0),
            new InstantCommand(()-> indexer.setPower(1.0)),
           new PrintCommand ("beggining"), new WaitCommand(1.0), new PrintCommand ("end"), 
           new InstantCommand(()-> indexer.setPower(0.0)))),
        
        new ParallelRaceGroup( new RunCommand(()-> drivetrain.setDrivePower(0.2)), new WaitCommand(2.0)), 
        new RunCommand(()-> drivetrain.tankDriveVolts(0.0, 0.0)),
        //shoots one ball and moves out tarmack 

        new ZeroPitch(turret),
        new ParallelRaceGroup(new UseIntake(intake), new SequentialCommandGroup(new WaitCommand(1.0)),
         new RunCommand(()-> drivetrain.setDrivePower(0.2)), 
         new WaitCommand(2.0)), new RunCommand(()-> drivetrain.tankDriveVolts(0.0, 0.0)),

         //zero's pitch, intake another ball and drives back to tarmack (used the same time so should land back at the same place?)

         new ParallelRaceGroup(new UpperHubShoot(turret), new SequentialCommandGroup(new WaitCommand(1.0),
            new InstantCommand(()-> indexer.setPower(1.0)),
           new PrintCommand ("beggining"), new WaitCommand(1.0), new PrintCommand ("end"), 
           new InstantCommand(()-> indexer.setPower(0.0)))),

           //repeat from above and index/shoots second ball

           new ParallelRaceGroup( new RunCommand(()-> drivetrain.setDrivePower(0.2)), new WaitCommand(2.0)), 
        new RunCommand(()-> drivetrain.tankDriveVolts(0.0, 0.0))

        //drives out tarmack again and stops

        );
        //fingers crossed

        //shoot, wait, taxi
        //Parallel Race Group: Runs multiple commands at the same time
        //Wait Command: waits (lol) in seconds
        //Run Command: runs the command that's given


        //intake ball and index and shoot parallel group
        //drive back

    }
}
