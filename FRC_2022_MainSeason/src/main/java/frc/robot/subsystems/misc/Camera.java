package frc.robot.subsystems.misc;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

public class Camera {
    private UsbCamera frontLivecam, backLivecam;

    public Camera(){
        frontLivecam = CameraServer.startAutomaticCapture(0);
        frontLivecam.setResolution(320, 240);
        frontLivecam.setFPS(15);
    }

    public void initializeFrontCamera() {
        frontLivecam.setResolution(320, 240);
        frontLivecam.setFPS(15);
    }

    public UsbCamera getFrontCamera(){
        return frontLivecam;
    }
}
