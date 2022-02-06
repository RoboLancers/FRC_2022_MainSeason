package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.PneumaticSystem;

public class UseCompressor extends CommandBase {
    private PneumaticSystem pneumatics;

    public UseCompressor(PneumaticSystem pneumatics) {
        this.pneumatics = pneumatics;
        addRequirements(pneumatics);
    }

    @Override
    public void execute() {
        pneumatics.regulateCompressor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
