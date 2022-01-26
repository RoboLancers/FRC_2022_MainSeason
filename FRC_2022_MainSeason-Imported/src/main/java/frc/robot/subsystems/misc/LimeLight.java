package frc.robot.subsystems.misc;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LimeLight implements Subsystem {
    private NetworkTable table;

    public LimeLight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget(){
        return table.getEntry("tv").getBoolean(false);
    }

    public double getYawOffset(){
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getPitchOffset(){
        return table.getEntry("ty").getDouble(0.0);
    }

    public void toggleLight(boolean on){
        table.getEntry("ledMode").setNumber(on ? 3 : 1);
    }
}
