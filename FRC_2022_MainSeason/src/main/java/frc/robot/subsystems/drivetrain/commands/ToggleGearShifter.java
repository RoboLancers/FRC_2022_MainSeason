package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drivetrain.enums.GearShifterState;
import frc.robot.subsystems.drivetrain.GearShifter;

public class ToggleGearShifter extends InstantCommand {
    
    private final GearShifter gearShifter;

    public ToggleGearShifter(final GearShifter gearShifter) {
        this.gearShifter = gearShifter;
        addRequirements(gearShifter);
    }

    @Override
    public void initialize() {
        //this.gearShifter.setGearShifter(gearShifter.getState() == GearShifterState.HIGHGEAR ? GearShifterState.LOWGEAR : GearShifterState.HIGHGEAR);
    }
}
