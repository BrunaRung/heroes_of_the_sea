/* Copyright (c) 2025 FIRST. All rights reserved. */
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * Odometry calibration op-mode + "DriveTrain" data log.
 *
 * <p><b>Procedure.</b> Drive the robot (robot-centric mecanum, gamepad1 sticks) from the start pose
 * to known field reference points: the audience corners (loading zones), the gates, and the middle
 * of the goal and audience walls. Line the robot's center up over each point, then press
 * <b>A (CAPTURE)</b>. Cycle which reference you are aiming at with <b>dpad left/right</b>.
 *
 * <p><b>Output.</b> A CSV, {@code Downloads/DriveTrain_<timestamp>.csv}, with one row per loop plus
 * a flag on capture rows. It contains everything needed to calibrate the correction scalars
 * (Localizer.FORWARD_SCALAR/STRAFE_SCALAR) and sanity-check heading: raw drive encoder ticks
 * (x4), the localizer pose, IMU yaw, velocity, the active multipliers, the driver sticks, and — on
 * capture rows — the selected reference point's KNOWN coordinates and the pose error there.
 *
 * <p>Reference coordinates are in Pedro coordinates (0..144 in). They are derived from field
 * geometry with the goal wall at Pedro y=144 (per the Fortunate Son field definition, goals on the
 * -X wall). Gate coordinates are placeholders - set them to the real DECODE gate positions.
 */
@TeleOp(name = "Odometry Calibration", group = "Calibration")
public class OdometryCalibrationOpMode extends LinearOpMode {

    /** Field reference points, Pedro coordinates (inches). Corners + wall midpoints are exact. */
    private enum Reference {
        AUDIENCE_CORNER_LEFT(0, 0),
        AUDIENCE_WALL_MID(72, 0),
        AUDIENCE_CORNER_RIGHT(144, 0),
        RIGHT_WALL_MID(144, 72),
        GOAL_CORNER_RIGHT(144, 144),
        GOAL_WALL_MID(72, 144),
        GOAL_CORNER_LEFT(0, 144),
        LEFT_WALL_MID(0, 72),
        GATE_LEFT(48, 72),    // TODO: real DECODE gate coordinates
        GATE_RIGHT(96, 72);   // TODO: real DECODE gate coordinates

        final double x, y;
        Reference(double x, double y) { this.x = x; this.y = y; }
    }

    static final double ROBOT_SIZE_INCHES = 18.0;

    private DcMotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private com.pedropathing.localization.Localizer localizer;
    private FtcDashboard dashboard;
    private DataLogger driveLogger;

    @Override
    public void runOpMode() {
        // Drive motors (same names/directions as the match TeleOp so calibration matches reality).
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, Localizer.LEFT_FRONT_MOTOR);
        leftBackDrive = hardwareMap.get(DcMotorEx.class, Localizer.LEFT_REAR_MOTOR);
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, Localizer.RIGHT_FRONT_MOTOR);
        rightBackDrive = hardwareMap.get(DcMotorEx.class, Localizer.RIGHT_REAR_MOTOR);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[]{leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Bulk-cache hub reads so 4 encoders + status cost one transfer per loop.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Localizer (drive encoders + IMU) and the Dashboard field overlay.
        localizer = new Localizer(hardwareMap, new Pose(72, 72, 0));
        dashboard = FtcDashboard.getInstance();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // DriveTrain calibration log.
        driveLogger = new DataLogger("DriveTrain");
        driveLogger.writeHeader(
                "marker,ref_name,ref_x,ref_y,"
                        + "pose_x,pose_y,pose_h_deg,imu_deg,"
                        + "lf_ticks,rf_ticks,lr_ticks,rr_ticks,"
                        + "vel_x,vel_y,vel_h_deg,"
                        + "stick_fwd,stick_strafe,stick_turn,"
                        + "err_x,err_y,err_dist,"
                        + Localizer.configCsvHeader());   // full localizer config, per row

        Reference reference = Reference.AUDIENCE_CORNER_LEFT;
        int captureCount = 0;
        boolean prevLeft = false, prevRight = false, prevA = false;

        telemetry.addData("Status", "Initialized - press START");
        telemetry.update();
        waitForStart();
        DataLogger.resetEpoch();

        while (opModeIsActive()) {
            // ------------------------- reference selection (edge-detected) -------------------------
            if (gamepad1.dpad_right && !prevRight) {
                reference = Reference.values()[(reference.ordinal() + 1) % Reference.values().length];
            }
            if (gamepad1.dpad_left && !prevLeft) {
                int n = Reference.values().length;
                reference = Reference.values()[(reference.ordinal() - 1 + n) % n];
            }
            boolean capture = gamepad1.a && !prevA;   // rising edge = capture this loop
            prevLeft = gamepad1.dpad_left;
            prevRight = gamepad1.dpad_right;
            prevA = gamepad1.a;
            if (capture) captureCount++;

            // ------------------------------- drive (robot-centric) --------------------------------
            double axial = -gamepad1.left_stick_y;
            double yaw = -gamepad1.left_stick_x;
            double lateral = -gamepad1.right_stick_x;
            double lfP = axial + lateral + yaw;
            double rfP = axial - lateral - yaw;
            double lbP = axial - lateral + yaw;
            double rbP = axial + lateral - yaw;
            double max = Math.max(Math.max(Math.abs(lfP), Math.abs(rfP)),
                    Math.max(Math.abs(lbP), Math.abs(rbP)));
            if (max > 1.0) { lfP /= max; rfP /= max; lbP /= max; rbP /= max; }
            leftFrontDrive.setPower(lfP);
            rightFrontDrive.setPower(rfP);
            leftBackDrive.setPower(lbP);
            rightBackDrive.setPower(rbP);

            // ------------------------------------ odometry ----------------------------------------
            localizer.update();
            Pose pose = localizer.getPose();
            Pose vel = localizer.getVelocity();
            double poseHDeg = Math.toDegrees(pose.getHeading());
            double imuDeg = Math.toDegrees(localizer.getIMUHeading());
            double errX = pose.getX() - reference.x;
            double errY = pose.getY() - reference.y;
            double errDist = Math.hypot(errX, errY);

            // -------------------------------------- log -------------------------------------------
            driveLogger.row()
                    .add(capture ? 1 : 0)
                    .add(reference.name()).add(reference.x).add(reference.y)
                    .add(pose.getX()).add(pose.getY()).add(poseHDeg).add(imuDeg)
                    .add(leftFrontDrive.getCurrentPosition()).add(rightFrontDrive.getCurrentPosition())
                    .add(leftBackDrive.getCurrentPosition()).add(rightBackDrive.getCurrentPosition())
                    .add(vel.getX()).add(vel.getY()).add(Math.toDegrees(vel.getHeading()))
                    .add(axial).add(lateral).add(yaw)
                    .add(errX).add(errY).add(errDist)
                    .add(Localizer.configCsv())   // full localizer config block (multiple columns)
                    .end();

            // ------------------------------ dashboard field overlay -------------------------------
            TelemetryPacket packet = new TelemetryPacket();
            Canvas c = packet.fieldOverlay();
            drawReference(c, reference);
            drawRobot(c, pose);
            dashboard.sendTelemetryPacket(packet);

            // --------------------------------- driver telemetry -----------------------------------
            telemetry.addLine("<h2>🎯 ODOMETRY CALIBRATION</h2>");
            telemetry.addLine(String.format(
                    "Target: <b>%s</b> (%.0f, %.0f)", reference.name(), reference.x, reference.y));
            telemetry.addLine(String.format(
                    "Pose: X %.1f  Y %.1f  H %.1f&deg;", pose.getX(), pose.getY(), poseHDeg));
            telemetry.addLine(String.format(
                    "Error to target: <b>%.1f in</b>  (dx %.1f, dy %.1f)", errDist, errX, errY));
            telemetry.addLine(String.format("Captures logged: <b>%d</b>", captureCount));
            telemetry.addLine("<small>dpad = target &nbsp; A = CAPTURE &nbsp; sticks = drive</small>");
            telemetry.update();
        }

        driveLogger.close();   // flush + copy CSV to Downloads
    }

    /** Draws the robot as an 18" square + heading tick, converting Pedro -> Dashboard frame. */
    private void drawRobot(Canvas overlay, Pose pedroPose) {
        Pose f = pedroPose.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
        double x = f.getX(), y = f.getY(), h = f.getHeading();
        double half = ROBOT_SIZE_INCHES / 2.0;
        double cos = Math.cos(h), sin = Math.sin(h);
        double[][] corners = {{half, half}, {half, -half}, {-half, -half}, {-half, half}};
        double[] xs = new double[4], ys = new double[4];
        for (int i = 0; i < 4; i++) {
            xs[i] = x + corners[i][0] * cos - corners[i][1] * sin;
            ys[i] = y + corners[i][0] * sin + corners[i][1] * cos;
        }
        overlay.setStroke("#4CAF50").setStrokeWidth(1).strokePolygon(xs, ys);
        overlay.setStroke("#FFFFFF").strokeLine(x, y, x + half * cos, y + half * sin);
    }

    /** Draws the selected reference point as a target marker, converting Pedro -> Dashboard frame. */
    private void drawReference(Canvas overlay, Reference reference) {
        Pose f = new Pose(reference.x, reference.y, 0).getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
        overlay.setStroke("#FF9800").setStrokeWidth(2);
        overlay.strokeCircle(f.getX(), f.getY(), 4);
        overlay.strokeLine(f.getX() - 6, f.getY(), f.getX() + 6, f.getY());
        overlay.strokeLine(f.getX(), f.getY() - 6, f.getX(), f.getY() + 6);
    }
}
