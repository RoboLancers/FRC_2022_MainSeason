package frc.robot.subsystems.drivetrain.enums;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public enum GearShifterState {
    HIGHGEAR(DoubleSolenoid.Value.kForward),
    LOWGEAR(DoubleSolenoid.Value.kReverse);

    private DoubleSolenoid.Value value;

    GearShifterState(DoubleSolenoid.Value value) {
        this.value = value;
    }

    public DoubleSolenoid.Value getValue() {
        return value;
    }
}