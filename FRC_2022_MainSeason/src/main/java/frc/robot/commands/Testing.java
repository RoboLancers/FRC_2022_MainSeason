package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.UpperHubShoot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.climber.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Testing extends CommandBase {

    private Intake intake;
    private Indexer indexer;
    private Turret turret;
    private Climber climber;

    public Testing(Intake intake, Indexer indexer, Turret turret, Climber climber) {
        this.intake = intake;
        this.indexer = indexer;
        this.turret = turret;
    }

    public void execute() {
        new SequentialCommandGroup(
            new ParallelRaceGroup(new RunCommand(() -> {
                intake.setPower(0.05);
              }, intake),
              new WaitCommand(2)),
            new PrintCommand("intake forward success"),
            new ParallelRaceGroup(new RunCommand(() -> {
                    intake.setPower(-0.05);
                }, intake),
                new WaitCommand(2)),
            new PrintCommand("intake backward success"),
            new ParallelRaceGroup(new RunCommand(() -> {
                    intake.setPower(0);
            }, intake),
            new PrintCommand("intake off"),
            new WaitCommand(2)),
            new ParallelRaceGroup(new RunCommand(() -> {
                indexer.setPower(0.05);    
            }, indexer), new WaitCommand(2)),
            new PrintCommand("indexer forward success"),
            new ParallelRaceGroup(new RunCommand(() -> {
                indexer.setPower(-0.05);    
            }, indexer), new WaitCommand(2)),
            new PrintCommand("indexer backward success"),
            new ParallelRaceGroup(new RunCommand(() -> {
                indexer.setPower(0);    
            }, indexer),
            new PrintCommand("indexer off"),
            new WaitCommand(2)),
            new ParallelRaceGroup(
                new UpperHubShoot(turret),
                new WaitCommand(2)),
            new PrintCommand("turret success"),
            new ParallelRaceGroup(new RunCommand(() -> {
                climber.climberMotor1.set(0.1);
                climber.climberMotor2.set(0.1);
            }, climber),
            new WaitUntilCommand(1)),
            new PrintCommand("climber up success"),
            /* new ParallelRaceGroup(new RunCommand(() -> {
                intake.toggleIntake();
                intake.toggleIntake();
            }, intake),
            new WaitUntilCommand(15)), */
            new ParallelRaceGroup(new RunCommand(() -> {
                climber.climberMotor1.set(-0.1);
                climber.climberMotor2.set(-0.1);
            }, climber),
            new WaitUntilCommand(1)),
            new PrintCommand("climber down success")
        );
    }
}
