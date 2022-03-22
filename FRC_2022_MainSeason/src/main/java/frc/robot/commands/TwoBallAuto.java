package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.UpperHubShot;
import frc.robot.subsystems.turret.commands.ZeroPitch;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto(Drivetrain drivetrain, Turret turret, Indexer indexer, Intake intake){
        addCommands(
            new ZeroPitch(turret),
            new ParallelRaceGroup(
                new RunCommand(() -> {
                    drivetrain.arcadeDrive(0.4, 0);
                    intake.setPower(Constants.Intake.kIntakePower);
                }),
                new WaitUntilCommand(() -> {
                    return turret.limelight.hasTarget() && Math.abs(turret.limelight.pitchOffset()) < 2;
                })
            ),
            new InstantCommand(() -> {
                drivetrain.arcadeDrive(0, 0);
                intake.setPower(0);
            }),
            new ParallelRaceGroup(
                new UpperHubShot(turret),
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    new InstantCommand(() -> {
                        indexer.setPower(1.0);
                    }),
                    new WaitCommand(1.5)
                )
            ),
            new InstantCommand(() -> {
                indexer.setPower(0);
            })
        );
        addRequirements(drivetrain, turret, indexer, intake);
    }
}
