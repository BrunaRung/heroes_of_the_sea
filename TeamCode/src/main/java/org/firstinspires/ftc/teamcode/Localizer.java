package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
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
 * system (the default
 * of {@code new Pose(x, y, heading)} is {@code PedroCoordinates}); see doc/coordinates-system.md.
 *
 * <p>It is a deliberate variant of PP's own {@code DriveEncoderLocalizer}: the pose-exponential
 * integration is identical, but the heading channel is driven by the <b>IMU</b> (clean absolute
 * yaw) instead of the drive-encoder turn estimate. The four drive encoders still supply x/y
 * translation. This is a better fit for a robot with low-resolution drive encoders (e.g. REV HD
 * Hex, 28 CPR) whose encoder-derived heading would otherwise be grainy.
 *
 * <p>Swap path: once odometry pods arrive, replace the construction of this class with a
 * {@code PinpointLocalizer} / {@code ThreeWheelIMULocalizer} / etc. Because every consumer only
 * talks to the {@link com.pedropathing.localization.Localizer} interface ({@code update()},
 * {@code getPose()}, ...), no calling code changes.
 */
public class Localizer implements com.pedropathing.localization.Localizer {
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

    private double forwardTicksToInches;
    private double strafeTicksToInches;

    /**
     * Creates the localizer starting at (0, 0) facing 0 heading.
     */
    public Localizer(HardwareMap map, DriveEncoderConstants constants,
                     String imuName, RevHubOrientationOnRobot imuOrientation) {
        this(map, constants, imuName, imuOrientation, new Pose());
    }

    /**
     * Creates the localizer starting at the given pose.
     *
     * @param map            the HardwareMap
     * @param constants      PP DriveEncoderConstants (motor names, encoder directions, multipliers)
     * @param imuName        configured name of the Control Hub IMU (usually "imu")
     * @param imuOrientation how the Control Hub is mounted on the robot
     * @param setStartPose   starting pose, in Pedro coordinates
     */
    public Localizer(HardwareMap map, DriveEncoderConstants constants,
                     String imuName, RevHubOrientationOnRobot imuOrientation,
                     Pose setStartPose) {
        forwardTicksToInches = constants.forwardTicksToInches;
        strafeTicksToInches = constants.strafeTicksToInches;

        leftFront = new Encoder(map.get(DcMotorEx.class, constants.leftFrontMotorName));
        leftRear = new Encoder(map.get(DcMotorEx.class, constants.leftRearMotorName));
        rightRear = new Encoder(map.get(DcMotorEx.class, constants.rightRearMotorName));
        rightFront = new Encoder(map.get(DcMotorEx.class, constants.rightFrontMotorName));

        leftFront.setDirection(constants.leftFrontEncoderDirection);
        leftRear.setDirection(constants.leftRearEncoderDirection);
        rightFront.setDirection(constants.rightFrontEncoderDirection);
        rightRear.setDirection(constants.rightRearEncoderDirection);

        imu = map.get(IMU.class, imuName);
        imu.initialize(new IMU.Parameters(imuOrientation));
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
     * Robot-relative change in position: x/y from the drive encoders, heading from the IMU.
     *
     * @return a 3x1 Matrix of [forward, strafe, heading] deltas.
     */
    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3, 1);
        // x / forward movement
        returnMatrix.set(0, 0, forwardTicksToInches * (leftFront.getDeltaPosition()
                + rightFront.getDeltaPosition() + leftRear.getDeltaPosition() + rightRear.getDeltaPosition()));
        // y / strafe movement
        returnMatrix.set(1, 0, strafeTicksToInches * (-leftFront.getDeltaPosition()
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
        return forwardTicksToInches;
    }

    @Override
    public double getLateralMultiplier() {
        return strafeTicksToInches;
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
