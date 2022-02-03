package frc.robot.subsystems.Indexer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.Indexer.ColorSensor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.I2C;

public class Indexer extends SubsystemBase {
        // Maintains the sensor closest to the intake
        public final ColorSensorV3 firstColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        // Maintains the sensor closest to the turret
        public final ColorSensorV3 secondColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
}
