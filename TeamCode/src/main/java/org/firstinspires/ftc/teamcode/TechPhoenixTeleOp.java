package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * TechPhoenix 37965 - BIOBUZZ driver control.
 * =========================== CONTROLS ===========================
 * GAMEPAD 1 - DRIVER
 *   Left stick            drive and strafe
 *   Right stick X         rotate
 *   Right bumper (hold)   precision mode, 35% speed
 *   Left bumper (hold)    heading lock: robot holds its current angle
 *   A                     toggle field-centric
 *   Options / Back        reset heading (point robot away from driver first)
 *   X                     zero-slide encoder (slide must be fully down)
 * GAMEPAD 2 - OPERATOR
 *   Right trigger         intake in
 *   Left trigger          intake out (manual unjam)
 *   Y                     toggle flywheels on/off
 *   A (hold)              fire, while the interlock allows it
 *   Dpad left / right     range down / up, 0.25 m steps
 *   Dpad up / down        slide preset up / down
 *   Left stick Y          manual slide
 *   X                     toggle gripper
 *   B                     panic stop
 * Run HARDWARE TEST first, every time you rewire anything.
 */
@SuppressWarnings("unused")
@TeleOp(name = "TechPhoenix TeleOp", group = "Competition")
public class TechPhoenixTeleOp extends LinearOpMode {

    private static final double PRECISION_SCALE = 0.35;
    private static final double STICK_DEADZONE  = 0.05;

    private static final double RANGE_STEP = 0.25;
    private static final double RANGE_MIN  = 0.50;
    private static final double RANGE_MAX  = 5.00;
    private static final double RANGE_DEFAULT = 2.00;

    private static final double TELEOP_LENGTH_SEC = 120.0;
    private static final double ENDGAME_WARNING_SEC = 30.0;

    private final RobotHardware robot = new RobotHardware();

    private final Toggle fieldCentric  = new Toggle(true);
    private final Toggle launcherOn    = new Toggle(false);
    private final Toggle gripperClosed = new Toggle(false);

    private final EdgeDetector dpadLeft  = new EdgeDetector();
    private final EdgeDetector dpadRight = new EdgeDetector();
    private final EdgeDetector dpadUp    = new EdgeDetector();
    private final EdgeDetector dpadDown  = new EdgeDetector();
    private final EdgeDetector zeroSlideBtn = new EdgeDetector();

    private double targetRange = RANGE_DEFAULT;
    private boolean headingLockActive = false;
    private double lockedHeading = 0;

    private boolean slidePresetMode = false;
    private int slidePresetIndex = 0;

    private String blockReason = "";
    private boolean wasReady = false;
    private boolean endgameWarned = false;

    private final ElapsedTime loopTimer  = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);

        telemetry.addLine("TechPhoenix 37965 - TeleOp ready");
        telemetry.addLine("Point robot AWAY from driver station before START.");
        telemetry.addLine("Slide fully DOWN at init.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.resetHeading();
        matchTimer.reset();
        loopTimer.reset();

        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            handleDrive();
            handleShooter();
            handleIntake();
            handleSlide();
            handleGripper();
            handlePanicStop();
            handleEndgameAlert();

            robot.updateFeeder();
            robot.updateLauncher();

            showTelemetry(loopMs);
        }

        robot.stopAll();
    }

    // ==================== DRIVE ====================
    private void handleDrive() {

        fieldCentric.update(gamepad1.a);

        if (gamepad1.options || gamepad1.back) {
            robot.resetHeading();
            gamepad1.rumbleBlips(1);
        }

        double y  = curve(-gamepad1.left_stick_y);
        double x  = curve( gamepad1.left_stick_x);
        double rx = curve( gamepad1.right_stick_x);

        double scale = gamepad1.right_bumper ? PRECISION_SCALE : 1.0;
        y *= scale; x *= scale; rx *= scale;

        // HEADING LOCK: hold left bumper and the robot captures its angle,
        // then fights back to it. Get bumped mid-shot and it straightens out
        // on its own. Manual rotation overrides and re-captures on release.
        double heading = robot.getHeading();
        if (gamepad1.left_bumper) {
            if (!headingLockActive) {
                headingLockActive = true;
                lockedHeading = heading;
            }
            if (Math.abs(rx) < 0.05) {
                double error = RobotHardware.wrapAngle(lockedHeading - heading);
                rx = Range.clip(error * RobotHardware.HEADING_KP,
                        -RobotHardware.HEADING_MAX_CORRECTION,
                        RobotHardware.HEADING_MAX_CORRECTION);
            } else {
                lockedHeading = heading;
            }
        } else {
            headingLockActive = false;
        }

        if (fieldCentric.get()) {
            robot.driveFieldCentric(y, x, rx);
        } else {
            robot.drive(y, x, rx);
        }
    }

    /** Cubing squashes small inputs while keeping full deflection at full power. */
    private double curve(double input) {
        if (Math.abs(input) < STICK_DEADZONE) return 0;
        return input * input * input;
    }

    // ==================== SHOOTER ====================
    private void handleShooter() {

        launcherOn.update(gamepad2.y);
        robot.setLauncherRunning(launcherOn.get());

        if (dpadLeft.rose(gamepad2.dpad_left)) {
            targetRange = Range.clip(targetRange - RANGE_STEP, RANGE_MIN, RANGE_MAX);
            gamepad2.rumbleBlips(1);
        }
        if (dpadRight.rose(gamepad2.dpad_right)) {
            targetRange = Range.clip(targetRange + RANGE_STEP, RANGE_MIN, RANGE_MAX);
            gamepad2.rumbleBlips(1);
        }

        robot.aimAtRange(targetRange);

        blockReason = robot.getLaunchBlockReason(targetRange, false);
        boolean ready = blockReason.isEmpty();

        // Buzz the operator the instant the shot becomes legal, so they never
        // have to look away from the field to check the screen.
        if (ready && !wasReady) gamepad2.rumbleBlips(1);
        wasReady = ready;

        if (gamepad2.a && ready) robot.requestShot();
    }

    // ==================== INTAKE ====================
    private void handleIntake() {
        boolean jammed = robot.updateIntake(gamepad2.right_trigger, gamepad2.left_trigger);
        if (jammed) gamepad2.rumble(150);
    }

    // ==================== SLIDE ====================
    private void handleSlide() {

        if (zeroSlideBtn.rose(gamepad1.x)) {
            robot.zeroSlide();
            slidePresetMode = false;
            gamepad1.rumbleBlips(2);
        }

        double manual = curve(-gamepad2.left_stick_y);

        // Any manual input drops out of preset mode immediately, so the
        // operator is never fighting the automation.
        if (Math.abs(manual) > 0.02) {
            slidePresetMode = false;
            robot.setSlideManual(manual);
            return;
        }

        if (dpadUp.rose(gamepad2.dpad_up)) {
            slidePresetIndex = Math.min(slidePresetIndex + 1,
                    RobotHardware.SLIDE_PRESETS.length - 1);
            robot.setSlidePreset(slidePresetIndex);
            slidePresetMode = true;
            gamepad2.rumbleBlips(1);
        }
        if (dpadDown.rose(gamepad2.dpad_down)) {
            slidePresetIndex = Math.max(slidePresetIndex - 1, 0);
            robot.setSlidePreset(slidePresetIndex);
            slidePresetMode = true;
            gamepad2.rumbleBlips(1);
        }

        if (slidePresetMode) {
            robot.holdSlidePreset();
        } else {
            robot.setSlideManual(0);
        }
    }

    private void handleGripper() {
        gripperClosed.update(gamepad2.x);
        robot.setGripper(gripperClosed.get());
    }

    // ==================== SAFETY ====================
    private void handlePanicStop() {
        if (gamepad2.b) {
            robot.stopAll();
            launcherOn.set(false);
            slidePresetMode = false;
        }
    }

    /** Buzz both controllers once at 30 seconds left, so nobody watches a clock. */
    private void handleEndgameAlert() {
        double remaining = TELEOP_LENGTH_SEC - matchTimer.seconds();
        if (!endgameWarned && remaining <= ENDGAME_WARNING_SEC) {
            endgameWarned = true;
            gamepad1.rumble(600);
            gamepad2.rumble(600);
        }
    }

    // ==================== TELEMETRY ====================
    private void showTelemetry(double loopMs) {
        double remaining = Math.max(0, TELEOP_LENGTH_SEC - matchTimer.seconds());

        telemetry.addData("TIME LEFT", "%.0f s%s", remaining,
                remaining <= ENDGAME_WARNING_SEC ? "   *** ENDGAME ***" : "");
        telemetry.addData("Loop", "%.1f ms", loopMs);
        telemetry.addData("Battery", "%.2f V", robot.getBatteryVolts());
        telemetry.addLine();

        telemetry.addData("SHOT", wasReady ? ">>> READY <<<" : blockReason);
        telemetry.addData("Range", "%.2f m%s", targetRange,
                RobotHardware.rangeInsideTable(targetRange) ? "" : "  (outside table)");
        telemetry.addData("Speed", "%.0f / %.0f",
                robot.getLauncherVelocity(), robot.getCommandedSpeed());
        telemetry.addData("Hood", "%.3f", robot.getCommandedHood());
        telemetry.addData("Shots", robot.getShotCount());
        telemetry.addLine();

        telemetry.addData("Drive", fieldCentric.get() ? "FIELD" : "ROBOT");
        telemetry.addData("Heading lock", headingLockActive ? "HOLDING" : "off");
        telemetry.addData("Heading", "%.1f deg", robot.getHeadingDegrees());
        telemetry.addLine();

        telemetry.addData("Intake", robot.getIntakeState());
        telemetry.addData("Intake amps", "%.1f", robot.getIntakeAmps());
        telemetry.addData("Slide", "%d ticks%s", robot.getSlidePosition(),
                slidePresetMode
                        ? "  [" + RobotHardware.SLIDE_PRESET_NAMES[slidePresetIndex] + "]"
                        : "");
        telemetry.addData("Gripper", gripperClosed.get() ? "closed" : "open");

        telemetry.update();
    }

    // ==================== HELPERS ====================

    /** Turns a held button into a single flip, so one press equals one toggle. */
    private static class Toggle {
        private boolean state;
        private boolean last = false;
        Toggle(boolean initial) { state = initial; }
        void update(boolean input) {
            if (input && !last) state = !state;
            last = input;
        }
        boolean get() { return state; }
        @SuppressWarnings("SameParameterValue")
        void set(boolean value) { state = value; }
    }

    /** Fires true only on the frame a button goes from released to pressed. */
    private static class EdgeDetector {
        private boolean last = false;
        boolean rose(boolean input) {
            boolean result = input && !last;
            last = input;
            return result;
        }
    }
}
