package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;
import com.pedropathing.util.NanoTimer;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Drive-encoder + Control Hub IMU localizer.
 *
 * <p>This implements Pedro Pathing's {@link com.pedropathing.localization.Localizer} interface and
 * reuses PP's {@link Pose}, {@link Vector}, {@link Encoder}, {@link Matrix} and {@link NanoTimer},
 * so it is a drop-in {@code Localizer} anywhere PP expects one. Poses are in the Pedro coordinate
 * system (the default of {@code new Pose(x, y, heading)} is {@code PedroCoordinates}); see
 * doc/coordinates-system.md.
 *
 * <p>It is a deliberate variant of PP's own {@code DriveEncoderLocalizer}: the pose-exponential
 * integration is identical, but the heading channel is driven by the <b>IMU</b> (clean absolute
 * yaw) instead of the drive-encoder turn estimate. The four drive encoders still supply x/y
 * translation. This is a better fit for a robot with low-resolution drive encoders (e.g. REV HD
 * Hex, 28 CPR) whose encoder-derived heading would otherwise be grainy.
 *
 * <p>All odometry/localization configuration and calibration lives in this class (see the
 * CONFIGURATION &amp; CALIBRATION block below). The tick-&gt;inch calibration values are
 * {@code public static} and the class is annotated {@link Config}, so they are editable live from
 * the FTC Dashboard (under "Localizer") while an OpMode runs.
 *
 * <p>Swap path: once odometry pods arrive, replace the construction of this class with a
 * {@code PinpointLocalizer} / {@code ThreeWheelIMULocalizer} / etc. Because every consumer only
 * talks to the {@link com.pedropathing.localization.Localizer} interface ({@code update()},
 * {@code getPose()}, ...), no calling code changes.
 */
@Config
public class Localizer implements com.pedropathing.localization.Localizer {

    // ################################### ODOMETRY / LOCALIZATION ###############################
    // This is the robot's Pedro Pathing Localizer (see doc/coordinates-system.md).
    // Poses are in the Pedro coordinate system. The current implementation fuses the drive-motor
    // encoders (x/y) with the Control Hub IMU (heading); swap it for a Pinpoint/dead-wheel
    // Localizer later without touching the OpMode.
    //
    // All localizer configuration and calibration (motor names, encoder directions, hub
    // orientation, tick->inch constants) lives below; the tick->inch calibration is
    // FTC Dashboard-tunable (class is @Config, edit under "Localizer").

    // =============================== CONFIGURATION & CALIBRATION ===============================
    // Drive motor names - must match the robot configuration on the Control Hub.
    public static final String LEFT_FRONT_MOTOR = "left_front_drive";
    public static final String LEFT_REAR_MOTOR = "left_back_drive";
    public static final String RIGHT_FRONT_MOTOR = "right_front_drive";
    public static final String RIGHT_REAR_MOTOR = "right_back_drive";
    public static final String IMU_NAME = "imu";

    // Encoder counting directions - flip one (FORWARD <-> REVERSE) if that wheel's distance
    // reads backwards during tuning.
    public static final double LEFT_FRONT_ENCODER_DIRECTION = Encoder.REVERSE;
    public static final double RIGHT_FRONT_ENCODER_DIRECTION = Encoder.FORWARD;
    public static final double LEFT_REAR_ENCODER_DIRECTION = Encoder.REVERSE;
    public static final double RIGHT_REAR_ENCODER_DIRECTION = Encoder.FORWARD;

    // Control Hub mounting orientation: logo BACKWARD / USB UP.
    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
    public static final RevHubOrientationOnRobot HUB_ORIENTATION =
            new RevHubOrientationOnRobot(HUB_LOGO_DIRECTION, HUB_USB_DIRECTION);

    // Encoder tick->inch calibration for REV UltraPlanetary 12:1 + HD Hex on 75mm wheels:
    //   28 CPR * 10.484 (true 12:1) / (pi * 2.953") ~= 31.6 counts/inch.
    private static final double COUNTS_PER_INCH = 28.0 * 10.484 / (2.953 * Math.PI);

    // FTC Dashboard-tunable correction scalars (live-editable under "Localizer"). COUNTS_PER_INCH
    // above is fixed by the hardware; what you actually calibrate is a unitless correction scalar,
    // nominally 1.0. Effective inches-per-tick = SCALAR / COUNTS_PER_INCH. After a 48" push, set
    // FORWARD_SCALAR = 48 / measured_forward_inches (and STRAFE likewise with a sideways push).
    public static double FORWARD_SCALAR = 1.0;
    public static double STRAFE_SCALAR = 1.0;

    /** Effective forward inches travelled per encoder tick (correction scalar applied). */
    public static double forwardTicksToInches() {
        return FORWARD_SCALAR / COUNTS_PER_INCH;
    }

    /** Effective strafe inches travelled per encoder tick (correction scalar applied). */
    public static double strafeTicksToInches() {
        return STRAFE_SCALAR / COUNTS_PER_INCH;
    }

    /**
     * CSV header fragment (no leading/trailing comma) naming every config/calibration constant,
     * in the same order as {@link #configCsv()}. Kept in lock-step with configCsv() so a data log
     * can record the full localizer configuration on every row.
     */
    public static String configCsvHeader() {
        return "fwd_scalar,strafe_scalar,fwd_ticks_to_in,strafe_ticks_to_in,counts_per_inch,"
                + "lf_motor,lr_motor,rf_motor,rr_motor,imu_name,"
                + "lf_enc_dir,rf_enc_dir,lr_enc_dir,rr_enc_dir,"
                + "hub_logo,hub_usb";
    }

    /**
     * CSV value fragment (no leading/trailing comma) of every config/calibration constant, in the
     * same order as {@link #configCsvHeader()}. The scalars and effective tick-&gt;inch values are
     * read live so Dashboard edits are captured per row.
     */
    public static String configCsv() {
        return FORWARD_SCALAR + "," + STRAFE_SCALAR + ","
                + forwardTicksToInches() + "," + strafeTicksToInches() + "," + COUNTS_PER_INCH + ","
                + LEFT_FRONT_MOTOR + "," + LEFT_REAR_MOTOR + "," + RIGHT_FRONT_MOTOR + ","
                + RIGHT_REAR_MOTOR + "," + IMU_NAME + ","
                + LEFT_FRONT_ENCODER_DIRECTION + "," + RIGHT_FRONT_ENCODER_DIRECTION + ","
                + LEFT_REAR_ENCODER_DIRECTION + "," + RIGHT_REAR_ENCODER_DIRECTION + ","
                + HUB_LOGO_DIRECTION + "," + HUB_USB_DIRECTION;
    }

    // =================================== STATE ===================================
    private Pose startPose;
    private Pose displacementPose;
    private Pose currentVelocity;
    private Matrix prevRotationMatrix;
    private final NanoTimer timer;
    private long deltaTimeNano;

    private final Encoder leftFront;
    private final Encoder rightFront;
    private final Encoder leftRear;
    private final Encoder rightRear;

    private final IMU imu;
    private double previousIMUYaw;      // last IMU yaw reading (radians), for per-update delta
    private double imuDeltaHeading;     // change in heading this update (radians, CCW+)

    private double totalHeading;

    /**
     * Creates the localizer starting at (0, 0) facing 0 heading.
     */
    public Localizer(HardwareMap map) {
        this(map, new Pose());
    }

    /**
     * Creates the localizer starting at the given pose. All hardware names, encoder directions,
     * hub orientation and calibration come from the CONFIGURATION &amp; CALIBRATION constants above.
     *
     * @param map          the HardwareMap
     * @param setStartPose starting pose, in Pedro coordinates
     */
    public Localizer(HardwareMap map, Pose setStartPose) {
        leftFront = new Encoder(map.get(DcMotorEx.class, LEFT_FRONT_MOTOR));
        leftRear = new Encoder(map.get(DcMotorEx.class, LEFT_REAR_MOTOR));
        rightRear = new Encoder(map.get(DcMotorEx.class, RIGHT_REAR_MOTOR));
        rightFront = new Encoder(map.get(DcMotorEx.class, RIGHT_FRONT_MOTOR));

        leftFront.setDirection(LEFT_FRONT_ENCODER_DIRECTION);
        leftRear.setDirection(LEFT_REAR_ENCODER_DIRECTION);
        rightFront.setDirection(RIGHT_FRONT_ENCODER_DIRECTION);
        rightRear.setDirection(RIGHT_REAR_ENCODER_DIRECTION);

        imu = map.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(HUB_ORIENTATION));
        imu.resetYaw();
        previousIMUYaw = 0.0;

        setStartPose(setStartPose);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        displacementPose = new Pose();
        currentVelocity = new Pose();
    }

    /**
     * @return the current pose estimate (Pedro coordinates).
     */
    @Override
    public Pose getPose() {
        return startPose.plus(displacementPose);
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    /**
     * Sets the previous-heading rotation matrix used by the pose exponential.
     */
    public void setPrevRotationMatrix(double heading) {
        prevRotationMatrix = new Matrix(3, 3);
        prevRotationMatrix.set(0, 0, Math.cos(heading));
        prevRotationMatrix.set(0, 1, -Math.sin(heading));
        prevRotationMatrix.set(1, 0, Math.sin(heading));
        prevRotationMatrix.set(1, 1, Math.cos(heading));
        prevRotationMatrix.set(2, 2, 1.0);
    }

    @Override
    public void setPose(Pose setPose) {
        displacementPose = setPose.minus(startPose);
        resetEncoders();
    }

    /**
     * Updates the pose and velocity estimates. Translation deltas come from the drive encoders;
     * the heading delta comes from the IMU. Global displacement is computed with the same
     * pose-exponential method PP uses.
     */
    @Override
    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        updateEncoders();

        // Heading delta from the IMU (absolute yaw, CCW+), wrapped to the shortest signed arc.
        double imuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        imuDeltaHeading = imuYaw - previousIMUYaw;
        while (imuDeltaHeading > Math.PI) imuDeltaHeading -= 2.0 * Math.PI;
        while (imuDeltaHeading <= -Math.PI) imuDeltaHeading += 2.0 * Math.PI;
        previousIMUYaw = imuYaw;

        Matrix robotDeltas = getRobotDeltas();
        Matrix globalDeltas;
        setPrevRotationMatrix(getPose().getHeading());

        Matrix transformation = new Matrix(3, 3);
        if (Math.abs(robotDeltas.get(2, 0)) < 0.001) {
            transformation.set(0, 0, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(0, 1, -robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 0, robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 1, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(2, 2, 1.0);
        } else {
            transformation.set(0, 0, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(0, 1, (Math.cos(robotDeltas.get(2, 0)) - 1.0) / robotDeltas.get(2, 0));
            transformation.set(1, 0, (1.0 - Math.cos(robotDeltas.get(2, 0))) / robotDeltas.get(2, 0));
            transformation.set(1, 1, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(2, 2, 1.0);
        }

        globalDeltas = Matrix.multiply(Matrix.multiply(prevRotationMatrix, transformation), robotDeltas);

        displacementPose = displacementPose.plus(
                new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
        double deltaTimeSeconds = deltaTimeNano / Math.pow(10.0, 9);
        currentVelocity = new Pose(
                globalDeltas.get(0, 0) / deltaTimeSeconds,
                globalDeltas.get(1, 0) / deltaTimeSeconds,
                globalDeltas.get(2, 0) / deltaTimeSeconds);

        totalHeading += globalDeltas.get(2, 0);
    }

    public void updateEncoders() {
        leftFront.update();
        rightFront.update();
        leftRear.update();
        rightRear.update();
    }

    public void resetEncoders() {
        leftFront.reset();
        rightFront.reset();
        leftRear.reset();
        rightRear.reset();
    }

    /**
     * Robot-relative change in position: x/y from the drive encoders (using the live,
     * Dashboard-tunable calibration), heading from the IMU.
     *
     * @return a 3x1 Matrix of [forward, strafe, heading] deltas.
     */
    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3, 1);
        // x / forward movement
        returnMatrix.set(0, 0, forwardTicksToInches() * (leftFront.getDeltaPosition()
                + rightFront.getDeltaPosition() + leftRear.getDeltaPosition() + rightRear.getDeltaPosition()));
        // y / strafe movement
        returnMatrix.set(1, 0, strafeTicksToInches() * (-leftFront.getDeltaPosition()
                + rightFront.getDeltaPosition() + leftRear.getDeltaPosition() - rightRear.getDeltaPosition()));
        // theta / turning — from the IMU, NOT the encoders
        returnMatrix.set(2, 0, imuDeltaHeading);
        return returnMatrix;
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return forwardTicksToInches();
    }

    @Override
    public double getLateralMultiplier() {
        return strafeTicksToInches();
    }

    /**
     * Heading is measured by the IMU, not derived from encoder ticks, so there is no turning
     * ticks-to-radians multiplier. Returns NaN to signal "not applicable".
     */
    @Override
    public double getTurningMultiplier() {
        return Double.NaN;
    }

    @Override
    public void resetIMU() {
        imu.resetYaw();
        previousIMUYaw = 0.0;
    }

    /**
     * @return the IMU's absolute heading estimate (radians), offset by the start heading.
     */
    @Override
    public double getIMUHeading() {
        return startPose.getHeading()
                + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(getPose().getX())
                || Double.isNaN(getPose().getY())
                || Double.isNaN(getPose().getHeading());
    }
}
