package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.misc.LimeLight;
import frc.robot.subsystems.turret.commands.MatchTargetYaw;
import frc.robot.util.SparkMaxWrapper;

public class Turret extends SubsystemBase {
    // Limelight mounted on the turret base
    public LimeLight limelight;

    // Motor used to control the yaw of the turret
    // Motor used to control the pitch of the turret
    // First motor used to control the speed of the flywheel
    // Second motor used to control the speed of the flywheel
    private SparkMaxWrapper yawMotor, pitchMotor, flywheelMotorA, flywheelMotorB;

    public DigitalInput yawLimitSwitch;
    public DigitalInput pitchLimitSwitch;

    // Recorded delta yaw on last MatchTargetYaw execution
    public double deltaYaw;

    // The maximum absolute error in yaw that the turret is willing to shoot with
    // The turret yaw has to match the heading of the target before it can shoot
    private static double yawErrorThreshold = 1.0;

    // The maximum absolute change in yaw with respect to one periodic update that the turret is willing to shoot with
    // The turret yaw has to have stopped before it can shoot
    private static double yawDerivativeThreshold = 1.0;

    public Turret(
        LimeLight limelight
    ){
        this.limelight = limelight;
        this.yawMotor = new SparkMaxWrapper(Constants.Turret.Ports.yawMotor);
        this.pitchMotor = new SparkMaxWrapper(Constants.Turret.Ports.pitchMotor);
        this.flywheelMotorA = new SparkMaxWrapper(Constants.Turret.Ports.flywheelMotorA);
        this.flywheelMotorB = new SparkMaxWrapper(Constants.Turret.Ports.flywheelMotorB);
        this.yawLimitSwitch = new DigitalInput(Constants.Turret.Ports.yawLimitSwitch);
        this.pitchLimitSwitch = new DigitalInput(Constants.Turret.Ports.pitchLimitSwitch);

        this.setDefaultCommand(new MatchTargetYaw(this));
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
        // Very naive implementation, may need some more logic since it uses 2 independent motors
        this.flywheelMotorA.setPower(power);
        this.flywheelMotorB.setPower(power);
    };

    // Determine whether the yaw heading and derivative are valid for shooting
    public boolean headingIsAligned(){
        return (
            // Target must be in limelight view
            this.limelight.hasTarget() &&
            // Yaw heading error must be within acceptable threshold
            Math.abs(this.limelight.yawOffset()) < Turret.yawErrorThreshold &&
            // Yaw derivative with respect to one MatchTargetYaw execution must be within acceptable threshold
            Math.abs(this.deltaYaw) < Turret.yawDerivativeThreshold
        );
    };
}
