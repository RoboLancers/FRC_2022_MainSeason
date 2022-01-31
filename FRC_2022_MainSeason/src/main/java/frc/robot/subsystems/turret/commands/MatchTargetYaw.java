package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

// Default command to match the limelight if visible otherwise seek
public class MatchTargetYaw extends CommandBase {
    private Turret turret;

    // Last recorded yaw on execution
    private double lastYaw;

    // The turn rate per execution to rotate when limelight is not visible
    private static double seekAdjustment = 0.01;

    // Boolean representation of the sign coefficient applied to the seek adjustment
    private boolean seekDirection = true;

    // The minimum turn angle on the yaw axis the turret can reach
    private static double minSafeAngle = -180.0;

    // The maximum turn angle on the yaw axis the turret can reach
    private static double maxSafeAngle = 180.0;

    // The coefficient applied to the error in angle
    private static double proportionalTurnCoefficient = 0.01;

    // The coefficient applied to the change in angle
    private static double derivativeTurnCoefficient = 0.00;

    public MatchTargetYaw(Turret turret){
        this.turret = turret;
        this.lastYaw = this.turret.getYaw();
    }

    @Override
    public void execute(){
        double yaw = this.turret.getYaw();
        if(this.turret.limelight.hasTarget()){
            double yawWithRespectToTurret = yaw + this.turret.limelight.yawOffset();
            if(yawWithRespectToTurret < MatchTargetYaw.minSafeAngle || yawWithRespectToTurret > MatchTargetYaw.maxSafeAngle){
                int turnAdjustSign = yawWithRespectToTurret < MatchTargetYaw.minSafeAngle ? 1 : -1;
                this.turret.setYawPower(turnAdjustSign * seekAdjustment);
            } else {
                this.turret.deltaYaw = yaw - this.lastYaw;
                this.turret.setYawPower(yaw * MatchTargetYaw.proportionalTurnCoefficient - this.turret.deltaYaw * MatchTargetYaw.derivativeTurnCoefficient);
            }
        } else {
            this.turret.setYawPower(seekDirection ? seekAdjustment : -seekAdjustment);
        }
    }

    @Override
    public void end(boolean interrupted){
        this.turret.setYawPower(0);
    }
}
