package frc.robot.subsystems.turret;

// The trajectory information needed to hit the target with certain constraints

public class LaunchTrajectory {
    public double theta;
    public double speed;

    public LaunchTrajectory(
        double theta,       // The angle of the determined trajectory [in radians]
        double speed        // The speed of the determined trajectory [in meters per second]
    ){
        this.theta = theta;
        this.speed = speed;
    }

    public static double estimateDistance(double deltaY, double thetaX, double thetaY){
        return deltaY / (Math.cos(thetaX * Math.PI / 180) * Math.tan(thetaY * Math.PI / 180));
    }

    // Calculate the trajectory to hit the target at a given angle alpha
    public static LaunchTrajectory usingAlphaImpact(
        double g,           // Universal acceleration due to gravity [in meters per second]
        double dx,          // Distance away from target on XZ plane [in meters]
        double dy,          // Distance away from target on Y axis [in meters]
        double alpha        // Angle of impact on target in degrees [in radians]
    ){
        // calculate velocity the final target would need to hit the start point if launched at angle alpha
        /*
            dx = vf cos(alpha) t
            -dy = vf sin(alpha) t + 0.5 g t^2
            ...
            vf = dx / cos(alpha) / sqrt(2 * (-dy - dx tan(alpha)) / g)
        */
        double vf = dx / Math.cos(alpha) / Math.sqrt(2 * (-dy - dx * Math.tan(alpha)) / g);

        // derive initial velocity squared using kinematics
        /*
            v^2 * sin(theta)^2 = vf^2 * sin(alpha)^2 - 2 g dy
        */
        double v2 = (vf * vf) * (Math.sin(alpha) * Math.sin(alpha)) - 2 * g * dy;

        // determine quadratic polynomial to calculate theta
        /*
            v^2 sin^2(theta) = vSquared
            v = x / cos(theta) / sqrt(2 * (dy - dx tan(theta)) / g)
            ...
            tan^2(theta) (g dx^2) + tan(theta) (2 * vSquared * dx) + (-2 * vSquared * dy) = 0
        */
        double qA = g * (dx * dx);
        double qB = 2 * v2 * dx;
        double qC = -2 * v2 * dy;
        double theta = Math.atan2(-qB + Math.sqrt((qB * qB) - 4 * qA * qC), 2 * qA);

        // plug in formulas for remaining unknown quantities
        double speed = dx / Math.cos(theta) / Math.sqrt(2 * (dy - dx * Math.tan(theta)) / g);

        return new LaunchTrajectory(theta, speed);
    }

    public static LaunchTrajectory usingPassThrough(
        double g,           // Universal acceleration due to gravity [in meters per second]
        double dx0,         // Distance away from control point on XZ plane [in meters]
        double dy0,         // Distance away from control point on Y axis [in meters]
        double dx1,         // Distance away from target on XZ plane [in meters]
        double dy1          // Distance away from target on Y axis [in meters]
    ){
        // derive theta by using same initial velocity and solving for angle that passes through the control point and target
        /*
            dx0 / cos(theta) / sqrt(2 * (dy0 - dx0 * tan(theta)) / g) = dx1 / cos(theta) / sqrt(2 * (dy1 - dx1 * tan(theta)) / g)
            dx0 * sqrt(dy1 - dx1 * tan(theta)) = dx1 * sqrt(dy0 - dx0 * tan(theta))
            dx0 / dx1 = sqrt(dy0 - dx0 * tan(theta)) / sqrt(dy1 - dx1 * tan(theta))
            u = (dx0 ** 2) / (dx1 ** 2) = (dy0 - dx0 * tan(theta)) / (dy1 - dx1 * tan(theta))
            ...
            tan(theta) = (u * dy1 - dy0) / (u * dx1 - dx0)
        */
        double u = (dx0 * dx0) / (dx1 * dx1);
        double theta = Math.atan((u * dy1 - dy0) / (u * dx1 - dx0));
        double speed = dx1 / Math.cos(theta) / Math.sqrt(2 * (dy1 - dx1 * Math.tan(theta)) / g);

        return new LaunchTrajectory(theta, speed);
    }
}
