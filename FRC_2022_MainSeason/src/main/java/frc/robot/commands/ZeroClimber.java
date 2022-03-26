package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.commands.ResetClimber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.UpperHubShoot;
import frc.robot.subsystems.turret.commands.ZeroPitch;

public class ZeroClimber extends SequentialCommandGroup {
    public ZeroClimber(Climber climber, Turret turret){
        addCommands(
            new ZeroPitch(turret),
            new ResetClimber(climber, climber.climberMotor1),
            new WaitCommand(2),
            new ResetClimber(climber, climber.climberMotor2)
        );
        addRequirements(climber);
    }
}
