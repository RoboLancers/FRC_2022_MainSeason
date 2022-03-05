package frc.robot.autonomous.routines;

//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
//import frc.robot.subsystems.drivetrain.command.ToggleGearShifter; is used to switch from slower to faster speed and vice versa (gear shifter covers motor, weird metal cylinder i believe)
//import frc.robot.subsystems.intake.Intake;
//import frc.robot.subsystems.intake.commands.UseIntake;
import edu.wpi.first.wpilibj2.command.WaitCommand; 


public class ShootOneBall extends SequentialCommandGroup {
    public ShootOneBall (Drivetrain drivetrain) {

        //TODO: add commands to shoot 
        addCommands(new RunCommand(()-> drivetrain.setDrivePower(0.5)));
        addCommands(new WaitCommand(1.0));
        addCommands(new RunCommand(()-> drivetrain.setDrivePower(0.0)));

        //shoot, wait, taxi



    }
    
}
