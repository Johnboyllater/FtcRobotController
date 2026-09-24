package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TechPhoenix 37965 - BIOBUZZ autonomous, 30 seconds.
 * SELECT YOUR PLAN DURING INIT, before pressing play:
 *   Dpad up / down    pick the plan
 *   Dpad left / right adjust shooting range
 *   X / B             alliance BLUE / RED (only affects which way we strafe)
 * THE PLANS
 *   PARK_ONLY     drive off the line and stop. Always works. Start here.
 *   SHOOT_PARK    shoot the preloads from the start position, then park.
 *   SHOOT_COLLECT shoot preloads, drive forward intaking, shoot again, park.
 * A reliable PARK_ONLY that scores every single match beats an ambitious
 * routine that works one time in four. Run PARK_ONLY at your first event
 * unless SHOOT_PARK has worked ten times in a row in practice.
 * All distances below are GUESSES. Measure your field and fix them.
 */
@SuppressWarnings("unused")
@Autonomous(name = "TechPhoenix Auto", group = "Competition")
public class TechPhoenixAuto extends LinearOpMode {

    // ---- Field distances in inches. MEASURE THESE. ----
    private static final double PARK_DISTANCE   = 26.0;
    private static final double COLLECT_FORWARD = 30.0;
    private static final double DRIVE_POWER     = 0.45;
    private static final double STRAFE_POWER    = 0.5;

    /** How long to wait for the flywheels to reach speed before giving up. */
    private static final double SPINUP_TIMEOUT = 3.0;
    /** Gap between shots, so the feeder fully resets and the wheels recover. */
    private static final double SHOT_INTERVAL = 0.8;
    private static final int PRELOAD_COUNT = 3;

    private static final double RANGE_STEP = 0.25;

    private enum Plan { PARK_ONLY, SHOOT_PARK, SHOOT_COLLECT }

    private final RobotHardware robot = new RobotHardware();
    private Plan plan = Plan.PARK_ONLY;
    private double shootRange = 2.00;
    private boolean blueAlliance = true;

    private final ElapsedTime autoTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);

        // ---- Selection loop, runs while you are on the init screen ----
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

        while (opModeInInit()) {
            if (gamepad1.dpad_up && !lastUp)     plan = cyclePlan(plan, 1);
            if (gamepad1.dpad_down && !lastDown) plan = cyclePlan(plan, -1);
            if (gamepad1.dpad_right && !lastRight) shootRange += RANGE_STEP;
            if (gamepad1.dpad_left && !lastLeft)   shootRange -= RANGE_STEP;
            if (gamepad1.x) blueAlliance = true;
            if (gamepad1.b) blueAlliance = false;

            shootRange = Math.max(0.5, Math.min(5.0, shootRange));

            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            telemetry.addLine("=== AUTONOMOUS SETUP ===");
            telemetry.addData("PLAN", plan);
            telemetry.addData("Alliance", blueAlliance ? "BLUE (X)" : "RED (B)");
            telemetry.addData("Shoot range", "%.2f m%s", shootRange,
                    RobotHardware.rangeInsideTable(shootRange) ? "" : "  OUTSIDE TABLE");
            telemetry.addLine();
            telemetry.addData("Battery", "%.2f V", robot.getBatteryVolts());
            if (robot.getBatteryVolts() < RobotHardware.MIN_BATTERY_VOLTS) {
                telemetry.addLine(">>> BATTERY LOW, SWAP BEFORE THE MATCH <<<");
            }
            telemetry.addLine();
            telemetry.addLine("Dpad up/down: plan   left/right: range");
            telemetry.update();
        }

        if (isStopRequested()) return;

        robot.resetHeading();
        autoTimer.reset();

        switch (plan) {
            case PARK_ONLY:     runParkOnly();     break;
            case SHOOT_PARK:    runShootPark();    break;
            case SHOOT_COLLECT: runShootCollect(); break;
        }

        robot.stopAll();

        telemetry.addLine("Autonomous complete");
        telemetry.addData("Shots fired", robot.getShotCount());
        telemetry.addData("Time used", "%.1f s", autoTimer.seconds());
        telemetry.update();
    }

    // ================================================================
    //  PLANS
    // ================================================================

    /** Drive off the line and stop. The floor of every autonomous. */
    private void runParkOnly() {
        status("Driving to park");
        robot.driveInches(PARK_DISTANCE, DRIVE_POWER, 5.0);
    }

    /** Shoot the preloads from where we start, then park. */
    private void runShootPark() {
        status("Spinning up");
        robot.setLauncherRunning(true);
        robot.aimAtRange(shootRange);

        if (!waitForSpinup()) {
            // Flywheels never reached speed. Park anyway rather than scoring nothing.
            status("Spinup failed, parking");
            robot.setLauncherRunning(false);
            robot.updateLauncher();
            robot.driveInches(PARK_DISTANCE, DRIVE_POWER, 5.0);
            return;
        }

        fireVolley(PRELOAD_COUNT);

        robot.setLauncherRunning(false);
        robot.updateLauncher();

        status("Parking");
        robot.driveInches(PARK_DISTANCE, DRIVE_POWER, 5.0);
    }

    /** Shoot preloads, drive forward collecting, shoot again, park. */
    private void runShootCollect() {
        status("Spinning up");
        robot.setLauncherRunning(true);
        robot.aimAtRange(shootRange);

        if (waitForSpinup()) {
            fireVolley(PRELOAD_COUNT);
        }

        // Drive forward with the intake running to pick up whatever is in our path.
        status("Collecting");
        robot.updateIntake(1.0, 0);
        robot.driveInches(COLLECT_FORWARD, 0.35, 5.0);
        robot.updateIntake(0, 0);

        // Back to the shooting spot. Range grew by how far we drove.
        status("Returning");
        robot.driveInches(-COLLECT_FORWARD, DRIVE_POWER, 5.0);

        robot.aimAtRange(shootRange);
        if (waitForSpinup()) {
            fireVolley(2);
        }

        robot.setLauncherRunning(false);
        robot.updateLauncher();

        // Only park if there is actually time. Running out mid-drive is worse
        // than stopping where we are.
        if (autoTimer.seconds() < 24.0) {
            status("Parking");
            robot.driveInches(PARK_DISTANCE, DRIVE_POWER, 4.0);
        }
    }

    // ================================================================
    //  BUILDING BLOCKS
    // ================================================================

    /** Hold until the flywheels reach speed. Returns false on timeout. */
    private boolean waitForSpinup() {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SPINUP_TIMEOUT) {
            robot.updateLauncher();
            robot.updateFeeder();
            telemetry.addData("Spinning up", "%.0f / %.0f",
                    robot.getLauncherVelocity(), robot.getCommandedSpeed());
            telemetry.update();
            if (robot.launcherAtSpeed()) return true;
            idle();
        }
        return false;
    }

    /**
     * Fire a number of shots, waiting for the wheels to recover between each.
     * Skips a shot rather than firing slow, because a slow shot misses and
     * wastes a game element.
     */
    private void fireVolley(int shots) {
        for (int i = 0; i < shots && opModeIsActive(); i++) {

            // Ignore the range check here: auto is a fixed, practised shot.
            String block = robot.getLaunchBlockReason(shootRange, true);
            ElapsedTime wait = new ElapsedTime();
            while (opModeIsActive() && !block.isEmpty() && wait.seconds() < 1.5) {
                robot.updateLauncher();
                robot.updateFeeder();
                block = robot.getLaunchBlockReason(shootRange, true);
                idle();
            }

            if (!block.isEmpty()) continue;   // still blocked, skip this one

            robot.requestShot();
            status("Shot " + (i + 1) + " of " + shots);

            ElapsedTime gap = new ElapsedTime();
            while (opModeIsActive() && gap.seconds() < SHOT_INTERVAL) {
                robot.updateFeeder();
                robot.updateLauncher();
                idle();
            }
        }
    }

    private Plan cyclePlan(Plan current, int direction) {
        Plan[] all = Plan.values();
        int i = (current.ordinal() + direction + all.length) % all.length;
        return all[i];
    }

    private void status(String text) {
        telemetry.addData("AUTO", text);
        telemetry.addData("Time", "%.1f s", autoTimer.seconds());
        telemetry.addData("Shots", robot.getShotCount());
        telemetry.update();
    }
}
