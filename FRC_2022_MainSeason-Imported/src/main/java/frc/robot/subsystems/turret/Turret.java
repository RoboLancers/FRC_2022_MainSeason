package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private LimeLight limelight;
    private CANSparkMax yawMotor, pitchMotor, flywheelMotorR, flywheelMotorB;

    // Yaw of the robot last frame
    private double lastYaw;
    // Boolean representation of the sign coefficient applied to the seek adjustment
    private boolean seekDirection = true;

    public Turret(
        LimeLight limelight,        // Limelight mounted on the turretMotor
        CANSparkMax yawMotor,             // Motor used to control the yaw of the turret
        CANSparkMax pitchMotor,           // Motor used to control the pitch of the turret
        CANSparkMax flywheelMotorA,       // First motor used to control the speed of the flywheel
        CANSparkMax flywheelMotorB        // Second motor used to control the speed of the flywheel
    ){
        this.limelight = limelight;
        this.yawMotor = yawMotor;
        this.pitchMotor = pitchMotor;
        this.flywheelMotorA = flywheelMotorA;
        this.flywheelMotorB = flywheelMotorB;
        this.lastYaw = this.getYaw();
    }

    public double getYaw(){}

    public void setYawPower(double power){};

    public double getPitch(){}

    public void setPitchPower(double power){};

    public double getFlywheelAverageSpeed(){}

    public void setFlywheelPower(double power){};

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
