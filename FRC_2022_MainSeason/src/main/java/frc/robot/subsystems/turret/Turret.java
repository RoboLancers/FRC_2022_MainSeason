package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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
        this.yawMotor = new SparkMaxWrapper(Constants.Turret.Ports.kYawPort);
        this.pitchMotor = new SparkMaxWrapper(Constants.Turret.Ports.kPitchPort);
        this.flywheelMotorA = new SparkMaxWrapper(Constants.Turret.Ports.kFlywheelPort1);
        this.flywheelMotorB = new SparkMaxWrapper(Constants.Turret.Ports.kFlywheelPort2);
        this.yawLimitSwitch = new DigitalInput(Constants.Turret.Ports.kYawLimitPort);
        this.pitchLimitSwitch = new DigitalInput(Constants.Turret.Ports.kPitchLimitPort);

        // Finn: You should implement smart motion control in the pitch and yaw commands. See:
        // SparkMaxPIDController pitchPIDController = pitchMotor.getPIDController();
        // pitchPIDController.setP(kP);
        // pitchPIDController.setI(kI);
        // pitchPIDController.setD(kD);
        // pitchPIDController.setFF(kFF); // Feedforward calculated in sysid
        // pitchPIDController.setIZone(kIZone); // I believe this is the tolerance. 
        // pitchPIDController.setOutputRange(-1, 1); // min and max powers
        // int smartMotionSlot = 0;
        // pitchPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        // pitchPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        // pitchPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        // pitchPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
        // // All of these constants can be tuned in smartdashboard as per https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
        // // Then, you can use methods like 
        // pitchPIDController.setReference(yawAngleToEncoderValue(desiredAngle, ControlType.kSmartMotion); // You'll need to make yawAngleToEncoderValue()
        // Soft limits can be applied to prevent exceeding the safe angles
        // This can also be easily applied to pitch
        this.setDefaultCommand(new MatchTargetYaw(this)); // This should be moved to RobotContainer
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
