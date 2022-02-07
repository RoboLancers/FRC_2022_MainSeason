package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.subsystems.flywheel.commands.SetFlywheelSpeed;
import frc.robot.subsystems.turret.subsystems.pitch.commands.SetPitch;

public class AdjustPitchAndFlywheel extends ParallelRaceGroup {
    public AdjustPitchAndFlywheel(Turret turret){
        addCommands(
            // ! make this account for kPitchMountAngle
            new SetPitch(() -> { return turret.launchTrajectory.theta + Constants.Turret.PhysicsInfo.kPitchMountAngle; }, turret.pitch),
            new SetFlywheelSpeed(() -> { return turret.launchTrajectory.speed; }, turret.flywheel),
            new WaitUntilCommand(() -> { return turret.isReadyToShoot(); })
        );
    }
}