package frc.robot.util;

// Makes SparkMax easier to use and removes error linting in non-windows computers
public class SparkMaxWrapper {
    private CANSparkMax motor;

    public SparkMaxWrapper(int port){
        motor = new CANSparkMax(port, CANSparkMax.MotorType.kBrushless);
    }

    public double getPosition(){
        // double check this
        return motor.getPosition();
    }

    public double getVelocity(){
        // double check this
        return motor.getVelocity();
    }

    public void setPower(double power){
        // double check this
        motor.set(power);
    }
}
