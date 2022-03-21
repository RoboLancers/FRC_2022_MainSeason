package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.commands.UpperHubShoot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.climber.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Testing extends CommandBase {

    private Intake intake;
    private Indexer indexer;
    private Turret turret;
    private Climber climber;
    private DoubleSolenoid.Value value;

    public Testing(Intake intake, Indexer indexer, Turret turret, Climber climber) {
        this.intake = intake;
        this.indexer = indexer;
        this.turret = turret;
        this.value = intake.retractionPiston.get();
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
                intake.toggleIntake();
            }, intake),
            new WaitCommand(15),
            new WaitUntilCommand(() -> (intake.retractionPiston.get() != value))),
            new ParallelRaceGroup(new RunCommand(() -> {
                intake.toggleIntake();
            }, intake),
            new WaitCommand(15),
            new WaitUntilCommand(() -> (intake.retractionPiston.get() == value))));
    }
}
