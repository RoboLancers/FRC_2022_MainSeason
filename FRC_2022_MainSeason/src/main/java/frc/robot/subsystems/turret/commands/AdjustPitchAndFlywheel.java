package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.turret.Turret;

public class AdjustPitchAndFlywheel extends ParallelRaceGroup {
    public AdjustPitchAndFlywheel(Turret turret){
        addCommands(
            new SetPitch(
                () -> {
                    return turret.trajectory.theta; // TODO: gear ratio math to convert degrees into motor ticks
                },
                turret.pitchMotor
            ),
            new SetFlywheelSpeed(
                () -> {
                    return turret.trajectory.speed;
                },
                turret.flywheelMotorA,
                turret.flywheelMotorB
            ),
            new WaitUntilCommand(
                () -> {
                    return turret.isAligned();
                }
            )
        );
    }
}