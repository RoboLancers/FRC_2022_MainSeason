package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    public CANSparkMax rollerMotor, indexerMotor;
    public Solenoid rectractionMotor;

    public Intake() {
        this.rollerMotor = new CANSparkMax(Constants.Intake.ROLLER_PORT, MotorType.kBrushless); //intake motor for roller
        this.indexerMotor = new CANSparkMax(Constants.Intake.INDEXER_PORT, MotorType.kBrushless); //transfer motor for indexer
        this.rectractionMotor = new Solenoid(PneumaticsModuleType.REVPH, Constants.Intake.RETRACTOR_CHANNEL);
    }

    public CANSparkMax getRollerMotor() {return rollerMotor;}

    public CANSparkMax getIndexerMotor() {return indexerMotor;}



}