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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

// Based on the sample: Basic: Omni Linear OpMode
@Config
@TeleOp(name = "TeleOp Control", group = "Teleop")

public class TeleOpControlLinearOpMode extends LinearOpMode {

    // Starting pose in Pedro coordinates (default corner origin). Change per your auto start.
    // NEAR start poses imported from the ThunderStrike 33535 "Fortunate Son" code
    // (fortunateson/util/Field.java, STARTING_POSES[..][1]). Those are defined in the FTC standard
    // field frame (inches, degrees): Red Near (-51, 51, 134deg), Blue Near (-51, -51, -134deg).
    // Convert to Pedro coordinates: RedNear -> (123, 123, 44deg), BlueNear -> (21, 123, 136deg).
    private final Pose startPoseRedNear =
            new Pose(-51, 51, Math.toRadians(134), FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    private final Pose startPoseBlueNear =
            new Pose(-51, -51, Math.toRadians(-134), FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    // FAR start poses imported from the "Fortunate Son" code (Field.java STARTING_POSES[..][3],
    // "Far Out") in the FTC standard field frame (inches, degrees): Red (63, 15, 180deg),
    // Blue (63, -15, 180deg). Convert to Pedro: RedFar -> (87, 9, 90deg), BlueFar -> (57, 9, 90deg).
    private final Pose startPoseRedFar =
            new Pose(63, 15, Math.toRadians(180), FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    private final Pose startPoseBlueFar =
            new Pose(63, -15, Math.toRadians(180), FTCCoordinates.INSTANCE)
                    .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    private final Pose startPose = new Pose(72, 72, Math.PI);

    // ------------------------------ MATCH SETUP (selected during INIT) --------------------------
    // Driver picks these in the INIT selection loop; they choose the localizer's start pose.
    private enum Alliance { RED, BLUE }
    private enum StartingPosition { NEAR, FAR }
    private Alliance alliance = Alliance.RED;
    private StartingPosition startingPosition = StartingPosition.NEAR;

    // The Localizer (typed as PP's interface so it can be swapped for a Pinpoint/dead-wheel
    // localizer later) and the hubs cached for bulk encoder reads.
    private com.pedropathing.localization.Localizer localizer;
    private List<LynxModule> allHubs;

    // FTC Dashboard field overlay - draws the robot each loop. Robot is drawn as an 18" square.
    static final double ROBOT_SIZE_INCHES = 18.0;
    private FtcDashboard dashboard;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime catatime = new ElapsedTime();

    // Declare drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Declare end-effector members
    private DcMotor intake = null;
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;
    //private DcMotor foot = null; // Not using Foot

    // motor power 1 = 100% and 0.5 = 50%
    // negative values = reverse ex: -0.5 = reverse 50%
    private double INTAKE_IN_POWER = 1.0;
    private double INTAKE_OUT_POWER = -0.9;
    private double INTAKE_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;

    /* Not using Foot
    private double FOOT_UP_POWER = 1.0;
    private double FOOT_DOWN_POWER = -0.85;
    private double FOOT_OFF_POWER = 0.0;
    private double footPower = FOOT_OFF_POWER;
    */

    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1.0;
    // FTC Dashboard-tunable (class is @Config): live feed-forward to hold the catapult down.
    public static double CATAPULT_HOLD_POWER = 0.1;

    private enum CatapultModes {UP, DOWN, HOLD}
    private CatapultModes pivotMode;

    /* Not using Foot
    private enum FootMode {UP, DOWN, BRAKE}
    private FootMode footmode;
    */

    /*
     * Code to run ONCE when the driver hits INIT (same as previous year's init())
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        intake = hardwareMap.get(DcMotor.class, "intake");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");
        /* Not using Foot
        foot = hardwareMap.get(DcMotor.class, "foot");
        */

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed
        // to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        // set direction of wheel motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // set direction of subsystem motors
        intake.setDirection(DcMotor.Direction.FORWARD); // Forward should INTAKE.
        catapult1.setDirection(DcMotor.Direction.REVERSE); // Backwards should pivot DOWN, or in the stowed position.
        catapult2.setDirection(DcMotor.Direction.FORWARD);
        //foot.setDirection(DcMotor.Direction.REVERSE); // Backwards should should stay UP, or in the stowed position

        // set initial subsystem behavior
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //foot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ODOMETRY / LOCALIZATION SETUP
        // Bulk-read all hub data once per loop so reading 4 drive encoders costs one transfer.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Localizer (drive encoders + Control Hub IMU). All config/calibration lives in
        // Localizer.java; poses are reported in Pedro coordinates.
        localizer = new Localizer(hardwareMap, startPose);

        // FTC Dashboard: robot is drawn on the field overlay each loop (see drawRobot()).
        dashboard = FtcDashboard.getInstance();

        // Driver Station telemetry in HTML mode for graphical match-setup feedback.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Match setup during INIT: driver selects alliance + starting position (blocks until START).
        selectMatchSetup();
        if (isStopRequested()) return;

        // Lock the chosen starting pose (Pedro coordinates) into the localizer.
        localizer.setStartPose(selectedStartPose());

        runtime.reset();
        catatime.reset();
        // run until the end of the match (driver presses STOP)
        // same as previous year's loop() code
        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;
        while (opModeIsActive()) {
            double max;

            // ---------------------------- ODOMETRY UPDATE ----------------------------
            // One call advances the localizer (drive encoders for x/y, IMU for heading).
            localizer.update();
            Pose robotPose = localizer.getPose();   // Pedro coordinates, inches + radians

            // Draw the robot on the FTC Dashboard field overlay (converted to the FTC frame).
            sendFieldOverlay(robotPose);

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //axial = speed, lateral = turn, yaw = strafe
            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double yaw = gamepad1.left_stick_x;
            double lateral = gamepad1.right_stick_x;

            boolean intakeInButton = gamepad1.left_trigger > 0.2;
            boolean intakeOutButton = gamepad1.left_bumper;

            // This conditional reduces ambiguity when multiple buttons are pressed.
            if (intakeInButton && intakeOutButton) {
                intakeInButton = false;
            }

            /* Not using Foot
            boolean footOutButton = gamepad1.a;
            boolean footUpButton = gamepad1.b;
            if (footOutButton && footUpButton) {
                footOutButton = false;
            }
            */

            boolean catapultUpButton = gamepad1.right_bumper;
            boolean catapultDownButton = gamepad1.right_trigger > 0.2;
            if (catapultUpButton && catapultDownButton) {
                catapultUpButton = false;
            }

            // DRIVE CODE
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is wheel test code
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

/*            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad */

            // INTAKE CODE
            if (intakeInButton) {
                intakePower = INTAKE_IN_POWER;
            } else if (intakeOutButton) {
                intakePower = INTAKE_OUT_POWER;
            } else {
                intakePower = INTAKE_OFF_POWER;
            }

            // FOOT CODE
            /* Not using Foot
            if (footOutButton) {
                footmode = FootMode.DOWN;
                footPower = FOOT_DOWN_POWER;
            } else if (footUpButton) {
                footmode = FootMode.UP;
                footPower = FOOT_UP_POWER;
            } else {
                footmode = FootMode.BRAKE;
                footPower = FOOT_OFF_POWER;
            }
            */

            // Determine pivot mode
            if (catapultUpButton) {
                pivotMode = CatapultModes.UP;
                catapult1.setPower(CATAPULT_UP_POWER);
                catapult2.setPower(CATAPULT_UP_POWER);
            } else if (catapultDownButton) {
                pivotMode = CatapultModes.DOWN;
                catapult1.setPower(CATAPULT_DOWN_POWER);
                catapult2.setPower(CATAPULT_DOWN_POWER);
            } else {
                pivotMode = CatapultModes.HOLD;
                catapult1.setPower(CATAPULT_HOLD_POWER);
                catapult2.setPower(CATAPULT_HOLD_POWER);
                //Slight feed forward to keep catapult down while driving
            }

            // WRITE EFFECTORS - Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            intake.setPower(intakePower);
            //foot.setPower(footPower); // Not using Foot

            String catapult_mode_str;
            if (pivotMode == CatapultModes.UP) {
                catapult_mode_str = "UP";
            } else if (pivotMode == CatapultModes.DOWN) {
                catapult_mode_str = "DOWN";
            } else {
                catapult_mode_str = "HOLD";
            }

            // UPDATE TELEMETRY
            // Show the elapsed game time, wheel power, and other systems power
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // Odometry pose in Pedro coordinates (see doc/coordinates-system.md).
            telemetry.addData("Pose (in, deg)", "X %5.1f  Y %5.1f  H %5.1f",
                    robotPose.getX(),
                    robotPose.getY(),
                    Math.toDegrees(robotPose.getHeading()));
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Intake", "%%4.2f", intake.getPower());
            //telemetry.addData("Foot Power", "%4.2f", foot.getPower()); // Not using Foot
            //telemetry.addData("Foot MODE", "%s", footmode); // Not using Foot
            telemetry.addData("Catapult1 Current/Target/power", "%d, %d, %4.2f",
                    catapult1.getCurrentPosition(), catapult1.getTargetPosition(), catapult1.getPower());
            telemetry.addData("Catapult2 Current/Target/power", "%d, %d, %4.2f",
                    catapult2.getCurrentPosition(), catapult2.getTargetPosition(), catapult2.getPower());
            telemetry.addData("Catapult MODE", "%s", catapult_mode_str);
            telemetry.update();
        }
    }

    /**
     * INIT-phase match-setup loop. Runs after INIT and before START; returns when START (or STOP)
     * is pressed. Edge-detected buttons on gamepad1:
     *   SHARE   -> toggle Alliance (RED / BLUE)
     *   OPTIONS -> toggle Starting Position (NEAR / FAR)
     * Graphical selection feedback is shown on the Driver Station via HTML telemetry.
     */
    private void selectMatchSetup() {
        boolean prevShare = false;
        boolean prevOptions = false;
        while (opModeInInit()) {
            if (gamepad1.share && !prevShare) {
                alliance = (alliance == Alliance.RED) ? Alliance.BLUE : Alliance.RED;
            }
            if (gamepad1.options && !prevOptions) {
                startingPosition = (startingPosition == StartingPosition.NEAR)
                        ? StartingPosition.FAR : StartingPosition.NEAR;
            }
            prevShare = gamepad1.share;
            prevOptions = gamepad1.options;

            // Reflect the current selection everywhere: robot start pose + field overlay.
            Pose selected = selectedStartPose();
            localizer.setStartPose(selected);
            sendFieldOverlay(selected);

            // Graphical selection feedback on the Driver Station.
            telemetry.addLine(matchSetupHtml());
            telemetry.update();
        }
    }

    /** @return the start pose (Pedro coordinates) for the selected alliance + starting position. */
    private Pose selectedStartPose() {
        if (alliance == Alliance.BLUE) {
            return startingPosition == StartingPosition.NEAR ? startPoseBlueNear : startPoseBlueFar;
        }
        return startingPosition == StartingPosition.NEAR ? startPoseRedNear : startPoseRedFar;
    }

    /** Alliance color (hex) used for both the HTML telemetry and the field overlay. */
    private String allianceColor() {
        return alliance == Alliance.RED ? "#e53935" : "#1e88e5";
    }

    /**
     * Converts a Pedro-coordinate pose to the FTC standard frame and draws the robot on the FTC
     * Dashboard field overlay, colored by the selected alliance.
     */
    private void sendFieldOverlay(Pose pedroPose) {
        // The FTC Dashboard field is oriented as Pedro's *inverted* FTC frame (180deg / point
        // reflection from FTCCoordinates), so use InvertedFTCCoordinates or the robot draws
        // mirrored on both axes.
        Pose fieldPose = pedroPose.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
        TelemetryPacket packet = new TelemetryPacket();
        drawRobot(packet.fieldOverlay(),
                fieldPose.getX(), fieldPose.getY(), fieldPose.getHeading(), allianceColor());
        dashboard.sendTelemetryPacket(packet);
    }

    /** Builds the graphical (HTML) match-setup readout for the Driver Station. */
    private String matchSetupHtml() {
        boolean red = alliance == Alliance.RED;
        String color = allianceColor();
        String square = red ? "🟥" : "🟦";   // red / blue large square
        boolean near = startingPosition == StartingPosition.NEAR;
        String robotOnSquare = "🤖" + square;          // robot on the alliance square
        String empty = "⬜";                                 // white square
        String nearCell = near ? robotOnSquare : empty;
        String farCell = near ? empty : robotOnSquare;
        return "<h2>🎮 MATCH SETUP</h2>"
                + "<h1><font color='" + color + "'>" + square + "&nbsp;" + alliance + " ALLIANCE</font></h1>"
                + "<big><b>Start:</b> <font color='" + color + "'>" + startingPosition + "</font></big>"
                + "<br><br>"
                + "<big>NEAR&nbsp;" + nearCell + "&nbsp;&nbsp;&nbsp;&nbsp;" + farCell + "&nbsp;FAR</big>"
                + "<br><br>"
                + "<small><b>SHARE</b> = Alliance&nbsp;&nbsp;|&nbsp;&nbsp;<b>OPTIONS</b> = Start"
                + "<br>Press <b>▶ START</b> to confirm</small>";
    }

    /**
     * Draws the robot on the FTC Dashboard field overlay as an {@link #ROBOT_SIZE_INCHES}-inch
     * square, with a line from the center to the middle of the front edge to show heading.
     *
     * @param overlay the field-overlay canvas
     * @param x       robot X, in inches, FTC standard field frame
     * @param y       robot Y, in inches, FTC standard field frame
     * @param heading robot heading, in radians (CCW+); the robot's front faces +x_robot
     * @param color   stroke color for the robot square (e.g. the alliance color)
     */
    private void drawRobot(Canvas overlay, double x, double y, double heading, String color) {
        double half = ROBOT_SIZE_INCHES / 2.0;
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Square corners in the robot frame (front is +x): front-left, front-right,
        // back-right, back-left. Rotate each by heading and translate to (x, y).
        double[][] robotCorners = {{half, half}, {half, -half}, {-half, -half}, {-half, half}};
        double[] xs = new double[4];
        double[] ys = new double[4];
        for (int i = 0; i < 4; i++) {
            xs[i] = x + robotCorners[i][0] * cos - robotCorners[i][1] * sin;
            ys[i] = y + robotCorners[i][0] * sin + robotCorners[i][1] * cos;
        }
        overlay.setStroke(color).setStrokeWidth(1).strokePolygon(xs, ys);

        // Heading line: robot center -> middle of the front edge (+x_robot).
        overlay.setStroke("#FFFFFF")
                .strokeLine(x, y, x + half * cos, y + half * sin);
    }
}