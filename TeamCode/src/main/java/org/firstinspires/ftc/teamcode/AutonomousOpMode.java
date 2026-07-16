/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * GOAL OF THIS OPMODE (DECODE autonomous, hardcoded straight-line path):
 *   1) Drive straight forward from the start position to a scoring position near the goal.
 *   2) Spin up and fire the catapult to launch the preloaded artifact(s) into the goal.
 *   3) Drive straight to a park position so the robot counts for the endgame parking bonus.
 * No turning, no strafing: both the intake and the catapult face the robot's front, the same
 * direction it drives, so driving straight at the goal already aims the shot — there is
 * intentionally no separate "aim" step. That means the robot's PLACEMENT heading at INIT is now
 * the only thing that aims the shot (see STARTING POSITION below) — there is no turn left in the
 * code to correct a bad placement, so getting the physical placement direction right matters more
 * than it would if a turn existed.
 *
 * HOW "DISTANCE" IS MEASURED (read this before tuning):
 *   The two drive segments are not "run for N milliseconds and hope." They use the same
 *   drive-encoder + Control Hub IMU Localizer that TeleOpControlLinearOpMode already uses for its
 *   Dashboard field overlay (see Localizer.java) to measure actual inches traveled, and stop the
 *   drive motors once the measured straight-line distance reaches the target you set below
 *   (DRIVE_TO_GOAL_DISTANCE_IN / PARK_DISTANCE_IN). You tune those with a tape measure, not by
 *   trial-and-error timing.
 *   Each drive segment ALSO keeps a millisecond timeout (DRIVE_TO_GOAL_TIMEOUT_MS /
 *   PARK_TIMEOUT_MS) as a safety net: if the localizer ever misbehaves (bad IMU read, a
 *   disconnected encoder, a wheel slipping in place so distance never accumulates) the segment
 *   still ends instead of driving forever into a wall. Round these up freely — they are NOT what
 *   determines where the robot ends up when things work normally, only DISTANCE_IN is, and that
 *   must be measured, not padded.
 *   The catapult spinup/fire steps are still pure time-based (CATAPULT_SPINUP_MS / FIRE_TIME_MS)
 *   because there is no sensor on the catapult that reports "launch complete."
 *   There is still NO heading correction while driving straight — if one side loses traction
 *   mid-drive the robot will veer even though the reported distance stays accurate, and (since
 *   there's no turn step anymore) nothing downstream corrects for that drift either.
 *
 * STARTING POSITION:
 *   The robot starts straddling the corner point shared by 4 tiles, not centered on a single
 *   tile, using a 144x144in field split into 6x6 24in tiles (columns A-F left-to-right along
 *   Pedro +X, rows 1-6 bottom-to-top along Pedro +Y). Two positions, mirrored across the field's
 *   center line (Pedro X=72):
 *     - RED:  corner shared by B5/B6/C5/C6 — Pedro coordinates (48, 120).
 *     - BLUE: corner shared by D5/D6/E5/E6 — Pedro coordinates (96, 120).
 *   The RED/BLUE labeling above is an arbitrary pick (the two positions are fully symmetric) — if
 *   it's backwards, just swap which corner each alliance's robot is physically placed at.
 *
 *   Because there is no code-side aim/turn step, the robot's FACING DIRECTION at placement is
 *   what determines whether the shot lines up with the goal — point the front (intake/catapult
 *   side) at the goal from that corner before pressing INIT. You do NOT need to know or set an
 *   exact numeric heading for the code to work (driveForDistance measures distance CHANGE from
 *   wherever the localizer happens to start, not an absolute compass angle) — what matters is
 *   placing the robot the SAME WAY every time (same corner, same facing direction) so the tuned
 *   DRIVE_TO_GOAL_DISTANCE_IN lands it in the same real-world spot, still aimed at the goal, match
 *   after match. Gaffer's tape marks on the practice field showing exactly where the
 *   wheels/corners go are the standard low-tech way teams make this repeatable.
 *
 *   STILL NEEDED FROM YOU: DRIVE_TO_GOAL_DISTANCE_IN — straight-line inches for the forward drive
 *   from that corner to the scoring spot.
 *
 * RECOMMENDED TUNING ORDER (retest after any hardware, battery, or field-surface change):
 *   1. DRIVE_POWER / DRIVE_TO_GOAL_DISTANCE_IN / DRIVE_TO_GOAL_TIMEOUT_MS - confirm the robot
 *      drives FORWARD (not backward), ends up actually facing the goal (this depends on physical
 *      placement, not code — see STARTING POSITION above), and stops at the scoring position.
 *   2. CATAPULT_SPINUP_MS / FIRE_TIME_MS - confirm the catapult is up to speed before FIRE_TIME_MS
 *      starts, and that FIRE_TIME_MS is long enough to fully launch every preloaded artifact.
 *   3. PARK_POWER / PARK_DISTANCE_IN / PARK_TIMEOUT_MS - confirm the robot ends fully inside the
 *      park zone, not short of it or overshooting past it.
 *
 * TIME BUDGET: the autonomous period is 30s and this whole sequence only needs a few seconds of
 * it. There is no need to trim DURATION/TIMEOUT constants down to a minimum — when in doubt,
 * round up. Worst case (every timeout gets hit) is well under 10s. This does NOT apply to
 * DRIVE_TO_GOAL_DISTANCE_IN / PARK_DISTANCE_IN — those must match reality, not be padded.
 */
@Autonomous(name = "Autonomous - Core", group = "Autonomous")
public class AutonomousOpMode extends LinearOpMode {

    // TUNE THIS: straight-line inches from the start position to the scoring position. Measure
    // this with a tape measure on the actual field (or practice field).
    private static final double DRIVE_TO_GOAL_DISTANCE_IN = 24.0;
    // TUNE THIS: safety cap — the drive segment ends if this much time passes even if the
    // localizer never reports reaching DRIVE_TO_GOAL_DISTANCE_IN. Round up freely; see TIME BUDGET
    // above.
    private static final long DRIVE_TO_GOAL_TIMEOUT_MS = 3000;
    // TUNE THIS: straight-drive power for the approach.
    private static final double DRIVE_POWER = 0.45;

    // TUNE THIS: short pause before firing so the catapult has time to spin up. Set generously —
    // the auto period is 30s and this sequence only needs a few seconds of it, so there's no
    // reason to shave this down to the minimum.
    private static final long CATAPULT_SPINUP_MS = 1000;
    // TUNE THIS: how long the catapult runs to fire the preloaded artifacts. Same reasoning —
    // round and generous beats precisely trimmed when time isn't the constraint.
    private static final long FIRE_TIME_MS = 1000;

    // TUNE THIS: straight-line inches from the scoring position to the park position.
    private static final double PARK_DISTANCE_IN = 18.0;
    // TUNE THIS: safety cap for the park segment. Round up freely; see TIME BUDGET above.
    private static final long PARK_TIMEOUT_MS = 3000;
    // TUNE THIS: parking drive power after the shot.
    private static final double PARK_POWER = 0.35;

    // TUNE THIS (only if the catapult misses its target): keep aligned with the existing TeleOp
    // catapult logic (CATAPULT_UP_POWER / CATAPULT_HOLD_POWER in TeleOpControlLinearOpMode) so
    // firing behavior is consistent between auto and teleop.
    private static final double CATAPULT_UP_POWER = -1.0;
    private static final double CATAPULT_HOLD_POWER = 0.1;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor intake = null;
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;

    // Drive-encoder + IMU localizer (see Localizer.java), the same one TeleOpControlLinearOpMode
    // uses. Used here only to measure straight-line distance traveled per segment, not for
    // field-absolute positioning or path-following.
    private Localizer localizer;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        telemetry.addData("Status", "Driving to goal");
        telemetry.update();
        driveForDistance(DRIVE_POWER, DRIVE_TO_GOAL_DISTANCE_IN, DRIVE_TO_GOAL_TIMEOUT_MS, "Driving to goal");

        telemetry.addData("Status", "Catapult spinup");
        telemetry.update();
        setCatapultPower(CATAPULT_UP_POWER);
        sleep(CATAPULT_SPINUP_MS);

        telemetry.addData("Status", "Firing");
        telemetry.update();
        runCatapultForTime(CATAPULT_UP_POWER, FIRE_TIME_MS, "Firing");

        setCatapultPower(CATAPULT_HOLD_POWER);

        telemetry.addData("Status", "Parking");
        telemetry.update();
        driveForDistance(PARK_POWER, PARK_DISTANCE_IN, PARK_TIMEOUT_MS, "Parking");

        stopAllMotors();
        telemetry.addData("Status", "Autonomous complete");
        telemetry.update();
    }

    private void initializeHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        intake = hardwareMap.get(DcMotor.class, "intake");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");

        // Match the existing drive setup from the TeleOp opmode (positive power = forward, per
        // TeleOpControlLinearOpMode's stick-to-power mapping). If the robot drives backward
        // when commanded forward, flip the direction of all four drive motors together.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        catapult1.setDirection(DcMotor.Direction.REVERSE);
        catapult2.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopAllMotors();

        // Localizer reads the same four drive motors' built-in encoders (RUN_WITHOUT_ENCODER above
        // only changes what motor.setPower() does internally, not whether getCurrentPosition()
        // reports ticks) plus the Control Hub IMU, so it works alongside the raw setPower() drive
        // control above without conflict. Start pose is arbitrary (0,0,0) since this OpMode only
        // ever measures relative distance traveled from a segment's own start, never an absolute
        // field position.
        localizer = new Localizer(hardwareMap);
    }

    /**
     * Drives straight at {@code power} until the localizer reports {@code targetDistanceIn}
     * inches of straight-line travel from where this call started, or {@code timeoutMs}
     * elapses — whichever comes first. See the class-level comment for why both exist.
     */
    private void driveForDistance(double power, double targetDistanceIn, long timeoutMs, String status) {
        runtime.reset();
        localizer.update();
        Pose segmentStart = localizer.getPose();
        setDrivePower(power);

        double traveledIn = 0.0;
        while (opModeIsActive() && traveledIn < targetDistanceIn && runtime.milliseconds() < timeoutMs) {
            localizer.update();
            if (localizer.isNAN()) {
                telemetry.addData("Status", status + " - localizer fault, stopping early");
                telemetry.update();
                break;
            }
            Pose current = localizer.getPose();
            traveledIn = Math.hypot(current.getX() - segmentStart.getX(), current.getY() - segmentStart.getY());

            telemetry.addData("Status", status);
            telemetry.addData("Traveled / Target (in)", "%.1f / %.1f", traveledIn, targetDistanceIn);
            telemetry.addData("Elapsed ms", runtime.milliseconds());
            telemetry.update();
            idle();
        }
        setDrivePower(0.0);
    }

    private void runCatapultForTime(double power, long durationMs, String status) {
        runtime.reset();
        setCatapultPower(power);
        while (opModeIsActive() && runtime.milliseconds() < durationMs) {
            telemetry.addData("Status", status);
            telemetry.addData("Elapsed ms", runtime.milliseconds());
            telemetry.update();
            idle();
        }
        setCatapultPower(CATAPULT_HOLD_POWER);
    }

    private void setDrivePower(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    private void setCatapultPower(double power) {
        catapult1.setPower(power);
        catapult2.setPower(power);
    }

    private void stopAllMotors() {
        setDrivePower(0.0);
        setCatapultPower(CATAPULT_HOLD_POWER);
        intake.setPower(0.0);
    }
}
