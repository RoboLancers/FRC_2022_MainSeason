package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.misc.AddressableLEDs;
import frc.robot.subsystems.turret.Turret;

public class UpdateLights extends CommandBase {
    public AddressableLEDs m_AddressableLEDs = new AddressableLEDs();

    public UpdateLights(Turret turret, Climber climber, Indexer indexer) {

        if (indexer.hasTwoBalls()) { //two cargo in indexer ; 
            m_AddressableLEDs.setTwoCargo();
            turretLEDCondition(turret); //will active turret led patterns if turret can be shot
        } else if (indexer.hasOneBall()) { // only one cargo ; yellow
            m_AddressableLEDs.setOneCargo();
            turretLEDCondition(turret);
        } else { // no cargo ; white
            m_AddressableLEDs.setNoCargo(); 
        }

        if (climber.isClung()) { //climber is hung ; crimson and gold
            m_AddressableLEDs.setHanging();
        }
    }

    public void turretLEDCondition(Turret turret) {
        if (turret.inShootingRange() && turret.isReadyToShoot()) { //hub is in range and turret ready to shoot and cargo in indexer; green
            m_AddressableLEDs.setTargetLocked();
        } else if (turret.isReadyToShoot()) { // center hub not in range ; red
            m_AddressableLEDs.setDistanceBad();
        }
            //m_AddressableLEDs.setFiringUp();

    }
}
