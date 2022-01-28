package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.misc.LimeLight;
import frc.robot.util.SparkMaxWrapper;

public class Turret extends SubsystemBase {
    // The turn rate per periodic to rotate when limelight is not visible
    private static double seekAdjustment = 0.01;

    // The minimum turn angle on the yaw axis the turret can reach
    private static double minSafeAngle = -180.0;

    // The maximum turn angle on the yaw axis the turret can reach
    private static double maxSafeAngle = 180.0;

    // The coefficient applied to the error in angle
    private static double proportionalTurnCoefficient = 0.01;

    // The coefficient applied to the change in angle
    private static double derivativeTurnCoefficient = 0.00;

    // Limelight mounted on the turret base
    private LimeLight limelight;

    // Motor used to control the yaw of the turret
    // Motor used to control the pitch of the turret
    // First motor used to control the speed of the flywheel
    // Second motor used to control the speed of the flywheel
    private SparkMaxWrapper yawMotor, pitchMotor, flywheelMotorA, flywheelMotorB;

    // Yaw of the robot last frame
    private double lastYaw;
    // Boolean representation of the sign coefficient applied to the seek adjustment
    private boolean seekDirection = true;

    public Turret(
        LimeLight limelight
    ){
        this.limelight = limelight;
        this.yawMotor = new SparkMaxWrapper(Constants.Turret.Ports.yawMotor);
        this.pitchMotor = new SparkMaxWrapper(Constants.Turret.Ports.pitchMotor);
        this.flywheelMotorA = new SparkMaxWrapper(Constants.Turret.Ports.flywheelMotorA);
        this.flywheelMotorB = new SparkMaxWrapper(Constants.Turret.Ports.flywheelMotorB);
        this.lastYaw = this.getYaw();
    }

    public double getYaw(){
        // TODO: MJ
        
        // silence errors until this is implemented
        return 0.0;
    }

    public void setYawPower(double power){
        // TODO: MJ
    };

    public double getPitch(){
        // TODO: NS

        // silence errors until this is implemented
        return 0.0;
    }

    public void setPitchPower(double power){
        // TODO: NS
    };

    public double getFlywheelAverageSpeed(){
        // TODO: NS

        // silence errors until this is implemented
        return 0.0;
    }

    public void setFlywheelPower(double power){
        // TODO: MP
    };

    public boolean isReadyToShoot(){
        // TODO: MP
        
        // silence errors until this is implemented
        return false;
    };

    // Handle seeking and matching target heading in background
    @Override
    public void periodic(){
        double yaw = this.getYaw();
        if(this.limelight.hasTarget()){
            double yawWithRespectToTurret = yaw + this.limelight.yawOffset();
            if(yawWithRespectToTurret < Turret.minSafeAngle || yawWithRespectToTurret > Turret.maxSafeAngle){
                int turnAdjustSign = yawWithRespectToTurret < Turret.minSafeAngle ? 1 : -1;
                this.setYawPower(turnAdjustSign * seekAdjustment);
            } else {
                double deltaYaw = yaw - lastYaw;
                this.setYawPower(yaw * Turret.proportionalTurnCoefficient - deltaYaw * Turret.derivativeTurnCoefficient);
            }
        } else {
            this.setYawPower(seekDirection ? seekAdjustment : -seekAdjustment);
        }
        lastYaw = yaw;
    }
}
