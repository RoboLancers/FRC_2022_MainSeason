public class Turret {
    // The turn rate per periodic to rotate when limelight is not visible
    private static double seekAdjustment = 0.0;

    // The minimum turn angle on the yaw axis the turret can reach
    private static double minSafeAngle = -180.0;

    // The maximum turn angle on the yaw axis the turret can reach
    private static double maxSafeAngle = 180.0;

    // The coefficient applied to the error in angle to adjust
    private static double turnAdjustCoefficient = 0.0;

    private Limelight limelight;
    private Motor yawMotor;
    private Motor pitchMotor;

    // Yaw of the robot last frame
    private double lastYaw;
    // Boolean representation of the sign coefficient applied to the seek adjustment
    private boolean seekDirection = true;

    public Turret(
        Limelight limelight,        // Limelight mounted on the turretMotor
        Motor yawMotor,             // Motor used to control the yaw of the turret
        Motor pitchMotor            // Motor used to control the pitch of the turret
    ){
        this.limelight = limelight;
        this.yawMotor = yawMotor;
        this.pitchMotor = pitchMotor;
        this.lastYaw = yawMotor.get();
    }

    // Handle seeking and matching target heading in background
    @Override
    public void periodic(){
        if(this.limelight.hasTarget()){
            double yawWithRespectToTurret = this.yawMotor.get() + this.limelight.getYawError();
            if(yawWithRespectToTurret < Turret.minSafeAngle || yawWithRespectToTurret > Turret.maxSafeAngle){
                int turnAdjustSign = yawWithRespectToTurret < Turn.minafeAngle ? 1 : -1;
                this.yawMotor.rotateBy(turnAdjustSign * seekAdjustment);
            } else {
                this.yawMotor.rotateBy(this.limelight.getYawError() * turnAdjustCoefficient);
            }
        } else {
            this.yawMotor.rotateBy(seekAdjustment * (turnAdjustCoefficient ? 1 : -1));
        }
    }
}
