package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TechPhoenix 37965 - HARDWARE TEST.
 * RUN THIS FIRST. Every time you rewire anything, change a config, or arrive
 * at a competition. It exercises one device at a time so you can see exactly
 * which one is wrong instead of guessing from a robot that drives sideways.
 * PUT THE ROBOT ON BLOCKS. Wheels must spin free.
 * CONTROLS
 *   Dpad up / down     select the device to test
 *   Right trigger      run the selected motor forward (or servo toward max)
 *   Left trigger       run it backward (or servo toward min)
 *   A                  reset all encoders to zero
 *   Y                  run ALL four drive motors forward together
 *   B                  emergency stop everything
 *   X                  sweep the selected servo between its limits
 * WHAT TO CHECK
 *   1. Each drive motor spins the direction its name says, with the robot
 *      viewed from above and the intake facing away from you.
 *   2. Each encoder counts UP when the motor runs forward. A count that goes
 *      down means the encoder cable is on the wrong port or the motor
 *      direction is reversed in code.
 *   3. Press Y. All four wheels should spin the same direction. If one is
 *      backwards, fix its setDirection line in RobotHardware, not the wiring.
 *   4. Each servo reaches both ends WITHOUT buzzing or straining. A buzzing
 *      servo is being driven past its travel and will burn out. Narrow that
 *      servo's MIN or MAX in RobotHardware until the buzzing stops.
 *   5. Watch the intake amps with the intake running free, then with your
 *      hand slowing it. Set INTAKE_JAM_AMPS just under the stalled figure.
 */
@SuppressWarnings("unused")
@TeleOp(name = "HARDWARE TEST", group = "Setup")
public class HardwareTest extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();

    private static final String[] DEVICES = {
            "frontLeft", "frontRight", "backLeft", "backRight",
            "intake", "slide", "launcherLeft", "launcherRight",
            "SERVO feeder", "SERVO hood", "SERVO gripper"
    };

    private int selected = 0;
    private double servoPos = 0.5;
    private boolean sweeping = false;
    private double sweepTarget = 1.0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);

        telemetry.addLine("HARDWARE TEST");
        telemetry.addLine();
        telemetry.addLine("*** PUT THE ROBOT ON BLOCKS ***");
        telemetry.addLine("Wheels must spin free before you press play.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        boolean lastUp = false, lastDown = false, lastA = false, lastX = false;

        while (opModeIsActive()) {

            // ---- Device selection ----
            if (gamepad1.dpad_up && !lastUp) {
                selected = (selected + 1) % DEVICES.length;
                stopAllTestOutputs();
                gamepad1.rumbleBlips(1);
            }
            if (gamepad1.dpad_down && !lastDown) {
                selected = (selected - 1 + DEVICES.length) % DEVICES.length;
                stopAllTestOutputs();
                gamepad1.rumbleBlips(1);
            }
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            // ---- Reset encoders ----
            if (gamepad1.a && !lastA) {
                resetAllEncoders();
                gamepad1.rumbleBlips(2);
            }
            lastA = gamepad1.a;

            // ---- Emergency stop ----
            if (gamepad1.b) {
                stopAllTestOutputs();
                sweeping = false;
            }

            // ---- All drive motors together ----
            if (gamepad1.y) {
                for (DcMotorEx m : robot.driveMotors()) m.setPower(0.3);
            } else if (!isServoSelected()) {
                runSelectedMotor();
            }

            // ---- Servo handling ----
            if (isServoSelected()) {
                if (gamepad1.x && !lastX) {
                    sweeping = !sweeping;
                    sweepTarget = 1.0;
                }
                lastX = gamepad1.x;
                handleServo();
            } else {
                sweeping = false;
            }

            showTelemetry();
        }

        stopAllTestOutputs();
    }

    // ================================================================

    private boolean isServoSelected() {
        return DEVICES[selected].startsWith("SERVO");
    }

    private DcMotorEx selectedMotor() {
        switch (selected) {
            case 0: return robot.frontLeft;
            case 1: return robot.frontRight;
            case 2: return robot.backLeft;
            case 3: return robot.backRight;
            case 4: return robot.intake;
            case 5: return robot.slide;
            case 6: return robot.launcherLeft;
            case 7: return robot.launcherRight;
            default: return null;
        }
    }

    private Servo selectedServo() {
        switch (selected) {
            case 8:  return robot.feeder;
            case 9:  return robot.hood;
            case 10: return robot.gripper;
            default: return null;
        }
    }

    private void runSelectedMotor() {
        DcMotorEx m = selectedMotor();
        if (m == null) return;

        double power = gamepad1.right_trigger - gamepad1.left_trigger;

        // Launchers get a lower test power. Full speed on blocks with no ball
        // is loud, throws debris, and wears the wheels for no reason.
        if (selected == 6 || selected == 7) power *= 0.4;

        m.setPower(Range.clip(power, -1.0, 1.0));
    }

    private void handleServo() {
        Servo s = selectedServo();
        if (s == null) return;

        if (sweeping) {
            // Slow sweep so you can watch for strain at the endpoints.
            servoPos += (sweepTarget > servoPos ? 0.004 : -0.004);
            if (Math.abs(servoPos - sweepTarget) < 0.01) {
                sweepTarget = (sweepTarget > 0.5) ? 0.0 : 1.0;
            }
        } else {
            servoPos += (gamepad1.right_trigger - gamepad1.left_trigger) * 0.004;
        }

        servoPos = Range.clip(servoPos, 0.0, 1.0);
        s.setPosition(servoPos);
    }

    private void stopAllTestOutputs() {
        for (DcMotorEx m : robot.driveMotors()) m.setPower(0);
        robot.intake.setPower(0);
        robot.slide.setPower(0);
        robot.launcherLeft.setPower(0);
        robot.launcherRight.setPower(0);
    }

    private void resetAllEncoders() {
        DcMotorEx[] all = {
                robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight,
                robot.intake, robot.slide, robot.launcherLeft, robot.launcherRight
        };
        for (DcMotorEx m : all) {
            DcMotor.RunMode previous = m.getMode();
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(previous == DcMotor.RunMode.RUN_USING_ENCODER
                    ? DcMotor.RunMode.RUN_USING_ENCODER
                    : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // ================================================================
    private void showTelemetry() {

        telemetry.addLine("===== HARDWARE TEST =====");
        telemetry.addData("TESTING", ">>> %s <<<", DEVICES[selected]);
        telemetry.addLine("Dpad = select   triggers = run   B = stop");
        telemetry.addLine();

        if (isServoSelected()) {
            telemetry.addData("Servo position", "%.3f", servoPos);
            telemetry.addData("Sweep", sweeping ? "ON (X to stop)" : "off (X to start)");
            telemetry.addLine("Listen for buzzing at the ends.");
            telemetry.addLine("Buzzing = past travel = narrow the limits.");
        } else {
            DcMotorEx m = selectedMotor();
            if (m != null) {
                telemetry.addData("Power", "%.2f", m.getPower());
                telemetry.addData("Encoder", m.getCurrentPosition());
                telemetry.addData("Velocity", "%.0f ticks/s", m.getVelocity());
                telemetry.addLine("Encoder must count UP when running forward.");
            }
        }

        telemetry.addLine();
        telemetry.addLine("--- ALL ENCODERS ---");
        telemetry.addData("FL / FR", "%d / %d",
                robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition());
        telemetry.addData("BL / BR", "%d / %d",
                robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
        telemetry.addData("slide", robot.getSlidePosition());
        telemetry.addLine();

        telemetry.addData("Intake amps", "%.2f", robot.getIntakeAmps());
        telemetry.addData("Heading", "%.1f deg", robot.getHeadingDegrees());
        telemetry.addData("Battery", "%.2f V", robot.getBatteryVolts());

        telemetry.update();
    }
}
