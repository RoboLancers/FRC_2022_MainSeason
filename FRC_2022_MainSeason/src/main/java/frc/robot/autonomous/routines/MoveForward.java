package frc.robot.autonomous.routines;
 
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.autonomous.*;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.UseDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.misc.LimeLight;
 
public class MoveForward extends SequentialCommandGroup {
    public MoveForward(Drivetrain drivetrain, Intake intake, LimeLight limelight, XboxController xboxController) {
        addCommands(new RunCommand(() -> drivetrain.tankDriveVolts(Constants.Intake.kLeftVolts, Constants.Intake.kRightVolts), drivetrain));
        addCommands(new WaitCommand(2.0));
        addCommands(new RunCommand(() -> drivetrain.tankDriveVolts(0, 0), drivetrain));

    }
}

//shoot, wait, taxi
//