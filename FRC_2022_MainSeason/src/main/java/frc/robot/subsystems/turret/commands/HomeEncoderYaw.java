package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

// Go to yaw motor home (-180) and reset encoder
public class HomeEncoderYaw extends CommandBase {
    private Turret turret;

    // Home direction of yaw motor is -180.0 relative to -180 to 180 reference
    private static double objectiveHomeDirection = -1.0;

    // The turn rate per execution to rotate to home position
    private static double seekAdjustment = 0.1;

    public HomeEncoderYaw(Turret turret){
        this.turret = turret;
    }

    @Override
    public void execute(){ 
        this.turret.setYawPower(HomeEncoderYaw.objectiveHomeDirection * HomeEncoderYaw.seekAdjustment);
    }

    @Override
    public void end(boolean interrupted){
        this.turret.setYawPower(0);
    }

    @Override
    public boolean isFinished(){
        return this.turret.yawLimitSwitch.get();
    }
}
