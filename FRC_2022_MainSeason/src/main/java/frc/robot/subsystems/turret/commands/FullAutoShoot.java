package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.subsystems.flywheel.commands.UseKickWheel;

public class FullAutoShoot extends SequentialCommandGroup {
    public FullAutoShoot(Turret turret){
        addCommands(
            new AdjustPitchAndFlywheel(turret),
            new UseKickWheel(turret.flywheel)
        );
    }
}
