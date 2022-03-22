package frc.robot.util;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxPID extends SubsystemBase {
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkMaxPIDController controller;

    public boolean usePID = true;
    public double setpointValue = 0.0;
    public ControlType setpointType = ControlType.kPosition;

    public DoubleSupplier kP = () -> 0;
    public DoubleSupplier kI = () -> 0;
    public DoubleSupplier kD = () -> 0;
    public DoubleSupplier kFF = () -> 0;

    public static DoubleSupplier useConstant(String name, double defaultValue, boolean tuningMode){
        if(tuningMode){
            SmartDashboard.putNumber(name, SmartDashboard.getNumber(name, defaultValue));
            return () -> SmartDashboard.getNumber(name, defaultValue);
        } else {
            return () -> defaultValue;
        }
    }

    public SparkMaxPID(int port, boolean inverted, double minVoltage, double maxVoltage){
        this.motor = new CANSparkMax(port, MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();
        this.controller = this.motor.getPIDController();

        this.motor.setInverted(inverted);
        this.motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        this.motor.enableSoftLimit(SoftLimitDirection.kForward, false);

        this.controller.setP(this.kP.getAsDouble());
        this.controller.setI(this.kI.getAsDouble());
        this.controller.setD(this.kD.getAsDouble());
        this.controller.setFF(this.kFF.getAsDouble());

        this.controller.setOutputRange(minVoltage, maxVoltage);
    }

    public SparkMaxPID(int port, boolean inverted, double minVoltage, double maxVoltage, DoubleSupplier kP, DoubleSupplier kI, DoubleSupplier kD, DoubleSupplier kFF){
        this.motor = new CANSparkMax(port, MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();
        this.controller = this.motor.getPIDController();

        this.motor.setInverted(inverted);
        this.motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        this.motor.enableSoftLimit(SoftLimitDirection.kForward, false);

        this.controller.setP(this.kP.getAsDouble());
        this.controller.setI(this.kI.getAsDouble());
        this.controller.setD(this.kD.getAsDouble());
        this.controller.setFF(this.kFF.getAsDouble());

        this.controller.setOutputRange(minVoltage, maxVoltage);
        
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;
    }

    @Override
    public void periodic(){
        if(this.usePID){
            this.controller.setP(this.kP.getAsDouble());
            this.controller.setI(this.kI.getAsDouble());
            this.controller.setD(this.kD.getAsDouble());
            this.controller.setFF(this.kFF.getAsDouble());
            this.controller.setReference(this.setpointValue, this.setpointType);
        } else {
            this.motor.set(this.setpointValue);
        }
    }

    public double getPosition(){
        return this.encoder.getPosition();
    }

    public double getVelocity(){
        return this.encoder.getVelocity();
    }

    public double getCurrent(){
        return this.motor.getOutputCurrent();
    }

    public void enableReverseLimit(double limit){
        this.motor.setSoftLimit(SoftLimitDirection.kReverse, (float) limit);
        this.motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    public void enableForwardLimit(double limit){
        this.motor.setSoftLimit(SoftLimitDirection.kForward, (float) limit);
        this.motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    public void disableReverseLimit(){
        this.motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    public void disableForwardLimit(){
        this.motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    }
}