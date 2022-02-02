package frc.robot.subsystems.misc;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LimeLight implements Subsystem {
    private NetworkTable table;

    public LimeLight(){
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget(){
        return this.table.getEntry("tv").getBoolean(false);
    }

    public double yawOffset(){
        return this.table.getEntry("tx").getDouble(0.0);
    }

    public double pitchOffset(){
        return this.table.getEntry("ty").getDouble(0.0);
    }
}
