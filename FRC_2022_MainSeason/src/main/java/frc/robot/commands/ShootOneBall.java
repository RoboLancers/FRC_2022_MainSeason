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
import frc.robot.subsystems.turret.commands.ActiveLaunchTrajectory;
import frc.robot.subsystems.turret.commands.UpperHubShoot;
import frc.robot.subsystems.turret.commands.ZeroPitch;
//import frc.robot.subsystems.drivetrain.command.ToggleGearShifter; is used to switch from slower to faster speed and vice versa (gear shifter covers motor, weird metal cylinder i believe)
//import frc.robot.subsystems.intake.Intake;
//import frc.robot.subsystems.intake.commands.UseIntake;
import edu.wpi.first.wpilibj2.command.WaitCommand;



public class ShootOneBall extends SequentialCommandGroup {
    public ShootOneBall (Drivetrain drivetrain, Turret turret, Indexer indexer) {
        
        addRequirements(indexer, drivetrain);

        addCommands(
            new ZeroPitch(turret),
           
            new ParallelRaceGroup(new UpperHubShoot(turret), new SequentialCommandGroup(new WaitCommand(1.0),
            new InstantCommand(()-> indexer.setPower(1.0)),
           new PrintCommand ("beggining"), new WaitCommand(1.0), new PrintCommand ("end"), 
           new InstantCommand(()-> indexer.setPower(0.0)))),
        
        new ParallelRaceGroup( new RunCommand(()-> drivetrain.setDrivePower(0.2)), new WaitCommand(2.0)), 
        new RunCommand(()-> drivetrain.tankDriveVolts(0.0, 0.0)));
        

        //shoot, wait, taxi
        //Parallel Race Group: Runs multiple commands at the same time
        //Wait Command: waits (lol) in seconds
        //Run Command: runs the command that's given

    }
}
