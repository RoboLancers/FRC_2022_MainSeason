package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.misc.LimeLight;

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
    private CANSparkMax yawMotor, pitchMotor, flywheelMotorR, flywheelMotorB;

    // Yaw of the robot last frame
    private double lastYaw;
    // Boolean representation of the sign coefficient applied to the seek adjustment
    private boolean seekDirection = true;

    public Turret(
        LimeLight limelight
    ){
        this.limelight = limelight;
        this.yawMotor = new CANSparkMax(Constants.Turret.Ports.yawMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.pitchMotor = new CANSparkMax(Constants.Turret.Ports.pitchMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.flywheelMotorA = new CANSparkMax(Constants.Turret.Ports.flywheelMotorA, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.flywheelMotorB = new CANSparkMax(Constants.Turret.Ports.flywheelMotorB, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.lastYaw = this.getYaw();
    }

    public double getYaw(){
        // TODO: MJ
    }

    public void setYawPower(double power){
        // TODO: MJ
    };

    public double getPitch(){
        // TODO: NS
    }

    public void setPitchPower(double power){
        // TODO: NS
    };

    public double getFlywheelAverageSpeed(){
        // TODO: NS
    }

    public void setFlywheelPower(double power){
        // TODO: MP
    };

    public boolean isReadyToShoot(){
        // TODO: MP
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
