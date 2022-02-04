package frc.robot.subsytems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake extends SubsystemBase{
    public CANSparkMax rollerMotor, indexerMotor;
    public Solenoid rectractionMotor;

    public Intake() {
        this.rollerMotor = new CANSparkMax(Constants.Intake.ROLLER_PORT, MotorType.kBrushless); //intake motor for roller
        this.indexerMotor = new CANSparkMax(Constants.Intake.TRANSFER_PORT, MotorType.kBrushless); //transfer motor for indexer
        this.rectractionMotor = new Solenoid(Constants.Intake.RETRACTOR_CHANNEL, Constants.Intake.RETRACTOR_CHANNEL);
    }

    public CANSparkMax getRollerMotor() {return rollerMotor;}

    public CANSparkMax getIndexerMotor() {return indexerMotor;}



}