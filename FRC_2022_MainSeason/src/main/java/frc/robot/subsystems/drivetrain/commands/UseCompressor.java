package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Pneumatics;

public class UseCompressor extends CommandBase {
    private Pneumatics pneumatics;

    public UseCompressor(Pneumatics pneumatics) {
        this.pneumatics = pneumatics;
        addRequirements(pneumatics);
    }

    // @Override
    // public void execute() {
    //     pneumatics.regulateCompressor();
    // }

    @Override
    public boolean isFinished() {
        return false;
    }
}
