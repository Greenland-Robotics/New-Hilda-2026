package gcsrobotics.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import gcsrobotics.control.OpModeBase;
import gcsrobotics.control.TeleOpBase;

@TeleOp(name = "ShooterTest", group = "Test")
public class ShooterTest extends TeleOpBase {

    private DcMotorEx left_shooter_wheel;
    private DcMotorEx right_shooter_wheel;
    private DcMotor intake;

    // Flywheel speed presets (RPM) — matching your 4 shot positions
    // TODO: replace FAR with tuned value once tested
    private static final double RPM_CLOSE  = 2400;
    private static final double RPM_MEDIUM = 3000;
    private static final double RPM_TOP    = 3600;
    private static final double RPM_FAR    = 4000; // TODO: tune

    // PIDF coefficients — matching Fenrir/Hilda config
    private static final double kP = 300;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kF = 10;

    @Override
    protected void inInit() {
        left_shooter_wheel  = hardwareMap.get(DcMotorEx.class, "left_shooter_wheel");
        right_shooter_wheel = hardwareMap.get(DcMotorEx.class, "right_shooter_wheel");
        intake              = hardwareMap.get(DcMotor.class, "intake");

        // Motor directions — TODO: confirm these match Hilda's physical config
        left_shooter_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        right_shooter_wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // Run flywheels using encoder for velocity control
        left_shooter_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_shooter_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply PIDF to both motors
        left_shooter_wheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        right_shooter_wheel.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        // Flywheels off at start
        left_shooter_wheel.setVelocity(0);
        right_shooter_wheel.setVelocity(0);
    }

    @Override
    protected void runLoop() {

        // ── Flywheel presets ──────────────────────────────────────
        // A = Close (2400), B = Medium (3000), X = Top (3600), Y = Far (4000)
        // Bumpers = stop flywheels
        if (gamepad1.a) {
            setFlywheelRPM(RPM_CLOSE);
        } else if (gamepad1.b) {
            setFlywheelRPM(RPM_MEDIUM);
        } else if (gamepad1.x) {
            setFlywheelRPM(RPM_TOP);
        } else if (gamepad1.y) {
            setFlywheelRPM(RPM_FAR);
        } else if (gamepad1.left_bumper || gamepad1.right_bumper) {
            setFlywheelRPM(0);
        }

        // ── Intake — left stick Y ─────────────────────────────────
        // Push forward = intake, pull back = reverse (unjam)
        intake.setPower(-gamepad1.left_stick_y);

        // ── Telemetry ─────────────────────────────────────────────
        telemetry.addData("Left flywheel RPM",  left_shooter_wheel.getVelocity());
        telemetry.addData("Right flywheel RPM", right_shooter_wheel.getVelocity());
        telemetry.addData("Intake power",       -gamepad1.left_stick_y);
        telemetry.addData("Controls", "A=Close  B=Medium  X=Top  Y=Far  Bumper=Stop");
        telemetry.update();
    }

    // Converts RPM to encoder ticks/sec and sets both motors
    private void setFlywheelRPM(double rpm) {
        // goBILDA Yellow Jacket: 28 ticks per revolution at motor shaft
        double ticksPerSec = rpm * 28.0 / 60.0;
        left_shooter_wheel.setVelocity(ticksPerSec);
        right_shooter_wheel.setVelocity(ticksPerSec);
    }
}
