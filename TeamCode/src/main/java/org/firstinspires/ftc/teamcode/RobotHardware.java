package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

/**
 * TechPhoenix 37965 - shared robot hardware.
 * Every OpMode builds one of these instead of declaring motors itself. Port
 * names, motor directions, servo limits and the range table live here and
 * ONLY here. Rewire something, fix it once, and all three OpModes are fixed.
 * The names in hardwareMap.get(...) must match your Robot Configuration on
 * the Driver Hub exactly, capital letters included.
 */
@SuppressWarnings("unused")
public class RobotHardware {
    // ================================================================
    //  RANGE TABLE  - { metres, flywheel ticks/sec, hood position }
    // ================================================================
    /**
     * Rows MUST be sorted by distance, smallest first. lookupRange()
     * interpolates between neighbouring rows, so tuning seven points gives
     * you every distance in between for free.
     * THESE ARE PLACEHOLDERS and will not work on your robot. Fill them in
     * with the Shooter Tuner OpMode.
     */
    public static final double[][] RANGE_TABLE = {
            //  metres   ticks/sec   hood
            {   1.00,      1250,     0.60 },
            {   1.50,      1450,     0.52 },
            {   2.00,      1650,     0.44 },
            {   2.50,      1850,     0.37 },
            {   3.00,      2050,     0.30 },
            {   3.50,      2250,     0.24 },
            {   4.00,      2450,     0.19 },
    };

    // ================================================================
    //  TUNING CONSTANTS
    // ================================================================

    // ---- Drivetrain geometry (for autonomous distances) ----
    /** goBILDA 5203 312 rpm. Change if you use a different motor. */
    public static final double TICKS_PER_MOTOR_REV = 537.7;
    /** 96 mm mecanum wheels. */
    public static final double WHEEL_DIAMETER_INCHES = 3.78;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double TICKS_PER_INCH =
            (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                    / (WHEEL_DIAMETER_INCHES * Math.PI);

    // ---- Launcher ----
    public static final double LAUNCHER_TOLERANCE = 80;
    public static final double LAUNCHER_MAX_TICKS = 2800;
    public static final double LAUNCHER_P = 10.0, LAUNCHER_I = 0.5,
            LAUNCHER_D = 0.0,  LAUNCHER_F = 12.0;
    /** Below this, shots go short no matter what the encoder reports. */
    public static final double MIN_BATTERY_VOLTS = 10.5;

    // ---- Feeder timing ----
    public static final double FEED_PUSH_TIME  = 0.18;
    public static final double FEED_RESET_TIME = 0.16;

    // ---- Servo endpoints (MEASURE THESE ON THE ROBOT) ----
    public static final double FEEDER_REST = 0.15, FEEDER_PUSH = 0.55;
    public static final double GRIPPER_OPEN = 0.20, GRIPPER_CLOSED = 0.65;
    public static final double HOOD_MIN = 0.10, HOOD_MAX = 0.75;

    // ---- Intake ----
    public static final double INTAKE_POWER = 1.0;
    public static final double INTAKE_JAM_AMPS = 6.0;
    public static final double INTAKE_JAM_TIME = 0.35;
    public static final double INTAKE_CLEAR_TIME = 0.30;

    // ---- Slide ----
    public static final double SLIDE_POWER = 0.8;
    public static final int SLIDE_MIN_TICKS = 0;
    public static final int SLIDE_MAX_TICKS = 1500;
    public static final int[] SLIDE_PRESETS = {0, 750, 1450};
    public static final String[] SLIDE_PRESET_NAMES = {"DOWN", "MID", "UP"};

    // ---- Heading control ----
    public static final double HEADING_KP = 1.6;
    public static final double HEADING_MAX_CORRECTION = 0.6;

    // ================================================================
    //  HARDWARE
    // ================================================================
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx intake, slide, launcherLeft, launcherRight;
    public Servo feeder, hood, gripper;
    public IMU imu;

    private HardwareMap hardwareMap;
    private LinearOpMode opMode;

    // ---- Launcher state ----
    private double commandedSpeed = 0;
    private double commandedHood = HOOD_MIN;
    private boolean launcherRunning = false;

    // ---- Feeder state ----
    public enum FeedState { IDLE, PUSHING, RESETTING }
    private FeedState feedState = FeedState.IDLE;
    private final ElapsedTime feedTimer = new ElapsedTime();
    private int shotCount = 0;

    // ---- Intake state ----
    public enum IntakeState { NORMAL, SUSPECT, CLEARING }
    private IntakeState intakeState = IntakeState.NORMAL;
    private final ElapsedTime intakeTimer = new ElapsedTime();

    // ================================================================
    //  INIT
    // ================================================================
    /** Call once at the top of runOpMode(). Pass the OpMode so autonomous
     *  helpers can check opModeIsActive() while they wait. */
    public void init(HardwareMap hwMap, LinearOpMode opMode) {
        this.hardwareMap = hwMap;
        this.opMode = opMode;

        // BULK CACHING: without this, every sensor read is its own round trip
        // to the hub. AUTO mode fetches everything in one trip and serves
        // cached values until you read the same thing twice. Typically, takes
        // a loop from ~50ms to ~10ms. Three lines, free speed.
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // ---- Drivetrain: Control Hub motors 0-3 ----
        frontLeft  = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hwMap.get(DcMotorEx.class, "backLeft");
        backRight  = hwMap.get(DcMotorEx.class, "backRight");

        // If the robot spins instead of driving straight, flip the wrong side
        // HERE, not by swapping wires. Wires get swapped back; code does not.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : driveMotors()) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // ---- Intake ----
        intake = hwMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---- Slide ----
        slide = hwMap.get(DcMotorEx.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---- Launcher: put these on the EXPANSION hub ----
        // Flywheel spin-up current can sag the rail the Control Hub's own
        // processor runs on, producing random disconnects that look exactly
        // like code bugs and waste weeks.
        launcherLeft  = hwMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hwMap.get(DcMotorEx.class, "launcherRight");

        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // RUN_USING_ENCODER commands a SPEED, not a raw power. setPower(0.7)
        // means 70 percent of whatever the battery has right now, so every
        // shot lands shorter than the last one across a match. This is the
        // single most important decision in the whole shooter.
        for (DcMotorEx m : new DcMotorEx[]{launcherLeft, launcherRight}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setVelocityPIDFCoefficients(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, LAUNCHER_F);
        }

        // ---- Servos ----
        feeder  = hwMap.get(Servo.class, "feeder");
        hood    = hwMap.get(Servo.class, "hood");
        gripper = hwMap.get(Servo.class, "gripper");

        double[] initial = lookupRange(2.0);
        commandedSpeed = initial[0];
        commandedHood  = clampHood(initial[1]);

        feeder.setPosition(FEEDER_REST);
        hood.setPosition(commandedHood);
        gripper.setPosition(GRIPPER_OPEN);

        // ---- IMU ----
        // Set these to match how the Control Hub is physically mounted, or
        // field-centric driving comes out rotated.
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
    }

    public DcMotorEx[] driveMotors() {
        return new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};
    }

    // ================================================================
    //  RANGE TABLE
    // ================================================================
    /** Linear interpolation between the two rows bracketing the given range.
     *  Returns { ticksPerSec, hoodPosition }. Clamps outside the table. */
    public static double[] lookupRange(double metres) {
        int n = RANGE_TABLE.length;

        if (metres <= RANGE_TABLE[0][0]) {
            return new double[]{RANGE_TABLE[0][1], RANGE_TABLE[0][2]};
        }
        if (metres >= RANGE_TABLE[n - 1][0]) {
            return new double[]{RANGE_TABLE[n - 1][1], RANGE_TABLE[n - 1][2]};
        }

        for (int i = 0; i < n - 1; i++) {
            double lo = RANGE_TABLE[i][0];
            double hi = RANGE_TABLE[i + 1][0];
            if (metres >= lo && metres <= hi) {
                double t = (metres - lo) / (hi - lo);
                return new double[]{
                        RANGE_TABLE[i][1] + t * (RANGE_TABLE[i + 1][1] - RANGE_TABLE[i][1]),
                        RANGE_TABLE[i][2] + t * (RANGE_TABLE[i + 1][2] - RANGE_TABLE[i][2])
                };
            }
        }
        return new double[]{RANGE_TABLE[0][1], RANGE_TABLE[0][2]};
    }

    public static boolean rangeInsideTable(double metres) {
        return metres >= RANGE_TABLE[0][0]
                && metres <= RANGE_TABLE[RANGE_TABLE.length - 1][0];
    }

    // ================================================================
    //  DRIVE
    // ================================================================
    /** Robot-centric mecanum. y forward, x right, rx clockwise. */
    public void drive(double y, double x, double rx) {
        x *= 1.1;  // strafing is mechanically weaker, so boost it
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        frontLeft.setPower ((y + x + rx) / denom);
        backLeft.setPower  ((y - x + rx) / denom);
        frontRight.setPower((y - x - rx) / denom);
        backRight.setPower ((y + x - rx) / denom);
    }

    /** Field-centric: stick direction is relative to the field, not the robot. */
    public void driveFieldCentric(double y, double x, double rx) {
        double heading = getHeading();
        double cos = Math.cos(-heading), sin = Math.sin(-heading);
        drive(x * sin + y * cos, x * cos - y * sin, rx);
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() { imu.resetYaw(); }

    /** Collapses any angle into -pi.pi so error maths never wraps wrong. */
    public static double wrapAngle(double radians) {
        while (radians >  Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    public void stopDrive() {
        for (DcMotorEx m : driveMotors()) m.setPower(0);
    }

    // ================================================================
    //  AUTONOMOUS DRIVE HELPERS
    // ================================================================
    /**
     * Drive straight a set distance using the drive encoders. Blocks until
     * done or the OpMode stops. Negative inches drives backwards.
     */
    public void driveInches(double inches, double power, double timeoutSec) {
        int ticks = (int) (inches * TICKS_PER_INCH);

        for (DcMotorEx m : driveMotors()) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(ticks);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(Math.abs(power));
        }

        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() < timeoutSec && anyDriveBusy()) {
            opMode.idle();
        }

        stopDrive();
        for (DcMotorEx m : driveMotors()) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Strafe sideways a set distance. Positive inches goes right.
     * Mecanum strafing slips, so treat this as approximate.
     */
    public void strafeInches(double inches, double power, double timeoutSec) {
        int ticks = (int) (inches * TICKS_PER_INCH);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(ticks);

        for (DcMotorEx m : driveMotors()) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(Math.abs(power));
        }

        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() < timeoutSec && anyDriveBusy()) {
            opMode.idle();
        }

        stopDrive();
        for (DcMotorEx m : driveMotors()) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Turn to an absolute heading in degrees using the IMU, with a simple
     * proportional controller. 0 is wherever the robot faced at init.
     */
    public void turnToHeading(double degrees, double timeoutSec) {
        double targetRad = Math.toRadians(degrees);
        ElapsedTime timer = new ElapsedTime();

        while (opMode.opModeIsActive() && timer.seconds() < timeoutSec) {
            double error = wrapAngle(targetRad - getHeading());
            if (Math.abs(Math.toDegrees(error)) < 2.0) break;

            double correction = Range.clip(error * HEADING_KP, -0.6, 0.6);
            // Minimum power, or friction stalls the robot just short.
            if (Math.abs(correction) < 0.12) {
                correction = Math.copySign(0.12, correction);
            }
            drive(0, 0, correction);
        }
        stopDrive();
    }
    @SuppressWarnings("UnusedReturnValue")
    private boolean anyDriveBusy() {
        for (DcMotorEx m : driveMotors()) {
            if (m.isBusy()) return true;
        }
        return false;
    }

    // ================================================================
    //  LAUNCHER
    // ================================================================
    public void setLauncherRunning(boolean on) { launcherRunning = on; }
    public boolean isLauncherRunning() { return launcherRunning; }

    /** Set speed and hood from the range table for a given distance. */
    public void aimAtRange(double metres) {
        double[] s = lookupRange(metres);
        setShot(s[0], s[1]);
    }

    /** Set speed and hood directly. Used by the tuner. */
    public void setShot(double ticksPerSec, double hoodPos) {
        commandedSpeed = Range.clip(ticksPerSec, 0, LAUNCHER_MAX_TICKS);
        commandedHood  = clampHood(hoodPos);
        hood.setPosition(commandedHood);
    }

    /** Push the commanded speed to the motors. Call every loop. */
    public void updateLauncher() {
        double target = launcherRunning ? commandedSpeed : 0;
        launcherLeft.setVelocity(target);
        launcherRight.setVelocity(target);
    }

    public double getCommandedSpeed() { return commandedSpeed; }
    public double getCommandedHood()  { return commandedHood; }

    public double getLauncherVelocity() {
        return (launcherLeft.getVelocity() + launcherRight.getVelocity()) / 2.0;
    }

    public boolean launcherAtSpeed() {
        if (commandedSpeed <= 0) return false;
        return Math.abs(getLauncherVelocity() - commandedSpeed) < LAUNCHER_TOLERANCE;
    }

    public double getBatteryVolts() {
        return hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public static double clampHood(double pos) {
        return Range.clip(pos, HOOD_MIN, HOOD_MAX);
    }

    // ================================================================
    //  LAUNCH INTERLOCK - the one place that decides if we may shoot
    // ================================================================
    /**
     * Empty string means firing is allowed. Otherwise, a short plain-language
     * reason. Every interlock lives here and nowhere else, so when the robot
     * refuses to shoot the driver station says exactly why.
     */
    public String getLaunchBlockReason(double targetRange, boolean ignoreRangeCheck) {
        if (!launcherRunning)             return "FLYWHEELS OFF - press Y";
        if (feedState != FeedState.IDLE)  return "feeder cycling";
        if (!launcherAtSpeed())           return "spinning up";
        if (getBatteryVolts() < MIN_BATTERY_VOLTS) return "BATTERY LOW - swap it";
        if (!ignoreRangeCheck && !rangeInsideTable(targetRange))
            return "range outside tuned table";
        return "";
    }

    // ================================================================
    //  FEEDER
    // ================================================================
    /** Request one shot. Returns true if the shot actually started. */
    @SuppressWarnings("UnusedReturnValue")
    public boolean requestShot() {
        if (feedState != FeedState.IDLE) return false;
        feeder.setPosition(FEEDER_PUSH);
        feedTimer.reset();
        feedState = FeedState.PUSHING;
        shotCount++;
        return true;
    }

    /** Advance the feeder state machine. Call every loop. */
    public void updateFeeder() {
        switch (feedState) {
            case PUSHING:
                if (feedTimer.seconds() > FEED_PUSH_TIME) {
                    feeder.setPosition(FEEDER_REST);
                    feedTimer.reset();
                    feedState = FeedState.RESETTING;
                }
                break;
            case RESETTING:
                if (feedTimer.seconds() > FEED_RESET_TIME) {
                    feedState = FeedState.IDLE;
                }
                break;
            default:
                break;
        }
    }

    public FeedState getFeedState() { return feedState; }
    public int getShotCount() { return shotCount; }

    public void resetFeeder() {
        feeder.setPosition(FEEDER_REST);
        feedState = FeedState.IDLE;
    }

    // ================================================================
    //  INTAKE with automatic jam clearing
    // ================================================================
    /**
     * A stalled intake draws a lot of current. If it stays high longer than a
     * brief spike, reverse briefly to spit the jam out. Clears most jams
     * before the operator notices one happened.
     *
     * @param in  forward command, 0 to 1
     * @param out manual reverse, 0 to 1, always wins
     * @return true if a jam clear just triggered (so callers can rumble)
     */
    public boolean updateIntake(double in, double out) {
        if (out > 0.1) {
            intake.setPower(-out * INTAKE_POWER);
            intakeState = IntakeState.NORMAL;
            return false;
        }

        double amps = intake.getCurrent(CurrentUnit.AMPS);
        boolean justJammed = false;

        switch (intakeState) {
            case NORMAL:
                intake.setPower(in > 0.1 ? in * INTAKE_POWER : 0);
                if (in > 0.1 && amps > INTAKE_JAM_AMPS) {
                    intakeTimer.reset();
                    intakeState = IntakeState.SUSPECT;
                }
                break;

            case SUSPECT:
                intake.setPower(in > 0.1 ? in * INTAKE_POWER : 0);
                if (amps < INTAKE_JAM_AMPS) {
                    intakeState = IntakeState.NORMAL;
                } else if (intakeTimer.seconds() > INTAKE_JAM_TIME) {
                    intakeTimer.reset();
                    intakeState = IntakeState.CLEARING;
                    justJammed = true;
                }
                break;

            case CLEARING:
                intake.setPower(-INTAKE_POWER);
                if (intakeTimer.seconds() > INTAKE_CLEAR_TIME) {
                    intakeState = IntakeState.NORMAL;
                }
                break;
        }
        return justJammed;
    }

    public IntakeState getIntakeState() { return intakeState; }

    public double getIntakeAmps() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    // ================================================================
    //  SLIDE
    // ================================================================
    /** Manual slide with soft limits, so the operator cannot grind the motor
     *  into a hard stop. */
    public void setSlideManual(double power) {
        int pos = slide.getCurrentPosition();
        if (power > 0 && pos >= SLIDE_MAX_TICKS) power = 0;
        if (power < 0 && pos <= SLIDE_MIN_TICKS) power = 0;

        if (slide.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        slide.setPower(power * SLIDE_POWER);
    }

    public void setSlidePreset(int index) {
        index = Range.clip(index, 0, SLIDE_PRESETS.length - 1);
        slide.setTargetPosition(SLIDE_PRESETS[index]);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(0.9);
    }

    public void holdSlidePreset() {
        if (!slide.isBusy()) slide.setPower(0.2);
    }

    public int getSlidePosition() { return slide.getCurrentPosition(); }

    /** Re-zero the slide encoder. Only with the slide physically all the way down. */
    public void zeroSlide() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setGripper(boolean closed) {
        gripper.setPosition(closed ? GRIPPER_CLOSED : GRIPPER_OPEN);
    }

    // ================================================================
    //  SAFETY
    // ================================================================
    public void stopAll() {
        stopDrive();
        intake.setPower(0);
        slide.setPower(0);
        launcherLeft.setVelocity(0);
        launcherRight.setVelocity(0);
        launcherRunning = false;
        resetFeeder();
        intakeState = IntakeState.NORMAL;
    }
}
