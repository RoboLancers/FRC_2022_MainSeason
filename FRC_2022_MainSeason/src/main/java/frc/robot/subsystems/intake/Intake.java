package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Intake extends SubsystemBase{
    public CANSparkMax rollerMotor, indexerMotor;
    public DoubleSolenoid rectractionMotor;

    public Intake() {
        rollerMotor = new CANSparkMax(Constants.Intake.ROLLER_PORT, MotorType.kBrushless); //intake motor for roller
        indexerMotor = new CANSparkMax(Constants.Intake.INDEXER_PORT, MotorType.kBrushless); //transfer motor for indexer
        rectractionMotor = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Intake.RETRACTOR_CHANNEL_ONE, Constants.Intake.RETRACTOR_CHANNEL_TWO);
    }

    public CANSparkMax getRollerMotor() {return rollerMotor;}

    public CANSparkMax getIndexerMotor() {return indexerMotor;}



}