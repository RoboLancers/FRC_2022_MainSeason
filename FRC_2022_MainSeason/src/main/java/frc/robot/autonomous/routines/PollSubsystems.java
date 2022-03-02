package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.ToggleGearShifter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.UseIntake;
import frc.robot.subsystems.misc.LimeLight;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.XboxController;

public class PollSubsystems extends SequentialCommandGroup{
    public PollSubsystems(Drivetrain drivetrain, Gyro gyro, Intake intake, LimeLight limelight) {
        addCommands(new UseIntake(intake)); // toggles the piston and activates rollers
        addCommands(new WaitCommand(1.0));
       // addCommands(new UseRollers(intake)); // turns off rollers
        addCommands(new WaitCommand(1.0));
        addCommands( );

    }
}
