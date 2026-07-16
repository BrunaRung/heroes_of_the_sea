package org.firstinspires.ftc.teamcode;

/**
 * Minimal 2D pose (inches, radians). Stands in for the Pedro Pathing Pose class that
 * AutonomousOpMode was originally written against — only what driveForDistance() needs.
 */
public class Pose {

    private final double x;
    private final double y;
    private final double heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}
