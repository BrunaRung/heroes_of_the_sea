package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Drive-encoder localizer used by AutonomousOpMode to measure straight-line distance traveled.
 *
 * It averages the four drive motors' built-in encoders and reports the distance driven as the
 * pose X coordinate (Y stays 0, heading stays 0). That is deliberately simple: AutonomousOpMode
 * only ever drives straight and takes the hypot() of the pose change since a segment started, so
 * "inches along the drive direction" is all it needs. This is NOT a field-absolute localizer and
 * won't track strafing or turning.
 *
 * Reading getCurrentPosition() works regardless of the motors being in RUN_WITHOUT_ENCODER —
 * that mode only changes what setPower() does internally, not whether ticks are reported.
 */
public class Localizer {

    // TUNE THESE to match your drivetrain, or measured distances will be off by a scale factor:
    //   TICKS_PER_MOTOR_REV: encoder ticks per output-shaft revolution of your drive motors
    //     (goBILDA 312 RPM Yellow Jacket = 537.7, REV HD Hex 20:1 = 560, REV Core Hex = 288).
    //   DRIVE_GEAR_RATIO: external gearing between motor shaft and wheel (1.0 if direct drive;
    //     >1.0 if the wheel turns slower than the motor).
    //   WHEEL_DIAMETER_IN: wheel diameter in inches (96mm goBILDA mecanum = 3.78).
    // Easy check: command a known distance, measure what the robot actually drove with a tape
    // measure, and scale these until reported and measured agree.
    private static final double TICKS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_RATIO = 1.0;
    private static final double WHEEL_DIAMETER_IN = 3.78;

    private static final double TICKS_PER_INCH =
            (TICKS_PER_MOTOR_REV * DRIVE_GEAR_RATIO) / (WHEEL_DIAMETER_IN * Math.PI);

    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final DcMotor rightFront;
    private final DcMotor rightBack;

    // Encoder readings at construction time, so the localizer starts at pose (0,0,0) no matter
    // what the raw tick counts happen to be.
    private final int leftFrontStart;
    private final int leftBackStart;
    private final int rightFrontStart;
    private final int rightBackStart;

    private Pose pose = new Pose(0.0, 0.0, 0.0);

    public Localizer(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBack = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontStart = leftFront.getCurrentPosition();
        leftBackStart = leftBack.getCurrentPosition();
        rightFrontStart = rightFront.getCurrentPosition();
        rightBackStart = rightBack.getCurrentPosition();
    }

    /** Re-reads the encoders and recomputes the pose. Call once per loop iteration. */
    public void update() {
        double leftFrontIn = (leftFront.getCurrentPosition() - leftFrontStart) / TICKS_PER_INCH;
        double leftBackIn = (leftBack.getCurrentPosition() - leftBackStart) / TICKS_PER_INCH;
        double rightFrontIn = (rightFront.getCurrentPosition() - rightFrontStart) / TICKS_PER_INCH;
        double rightBackIn = (rightBack.getCurrentPosition() - rightBackStart) / TICKS_PER_INCH;

        double distanceIn = (leftFrontIn + leftBackIn + rightFrontIn + rightBackIn) / 4.0;
        pose = new Pose(distanceIn, 0.0, 0.0);
    }

    /** Pose as of the last update(): X = inches driven since construction, Y and heading = 0. */
    public Pose getPose() {
        return pose;
    }

    /** True if the computed pose ever became NaN (shouldn't happen with encoder-only math). */
    public boolean isNAN() {
        return Double.isNaN(pose.getX()) || Double.isNaN(pose.getY());
    }
}
