package gcsrobotics.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import gcsrobotics.commands.SetHoodAngle;
import gcsrobotics.commands.ShootCurrent;
import gcsrobotics.commands.ShootingPosition;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.control.TeleOpBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.ButtonAction;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.SeriesCommand;

// =====================================================
// FlywheelTuner — PIDF tuning OpMode
//
// PURPOSE: Dial in F and P for each ShootingPosition,
//          validate 3-artifact burst recovery, and
//          confirm fire window tolerances before competition.
//
// WORKFLOW:
//   1. Start OpMode — flywheel spins to TOP velocity, hood moves to TOP.
//   2. Adjust F (D-pad left/right) until steady-state RPM ≈ target.
//   3. Adjust P (D-pad up/down) until burst recovery is fast, no oscillation.
//   4. Fire burst (right bumper) — watch recovery time on telemetry.
//   5. Switch positions (A/B/Y) and repeat.
//   6. Copy final F and P values into Constants.Flywheel and rebuild.
//
// CONTROLS — GP2 ONLY:
//   A               → CLOSE position (velocity + hood)
//   Y               → TOP position baseline (velocity + hood)
//   B               → FAR position (velocity + hood)
//   Right bumper     → Fire 3-artifact burst (velocity-gated)
//   Left bumper      → Stop flywheel
//   D-pad left       → Decrease F by current step
//   D-pad right      → Increase F by current step
//   D-pad down       → Decrease P by current step
//   D-pad up         → Increase P by current step
//   X                → Cycle step size (10 → 1 → 0.1 → 0.01)
// =====================================================

@TeleOp(name = "Flywheel Tuner", group = "Tuning")
public class FlywheelTuner extends TeleOpBase {

    // ---- Step sizes for F and P adjustment ----
    private static final double[] STEPS = {10.0, 1.0, 0.1, 0.01};
    private int stepIndex = 1; // start at 1.0

    // ---- Live PIDF values — start from Constants ----
    private double liveF = Constants.Flywheel.PIDF_F;
    private double liveP = Constants.Flywheel.PIDF_P;

    // ---- Current shooting position ----
    private ShootingPosition currentPosition = ShootingPosition.TOP;

    // ---- Button actions ----
    private ButtonAction posClose;
    private ButtonAction posTop;
    private ButtonAction posFar;
    private ButtonAction fireAction;
    private ButtonAction stopAction;
    private ButtonAction cycleStep;

    // ---- D-pad edge detection (no ButtonAction needed — direct state) ----
    private boolean prevDpadLeft  = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadUp    = false;
    private boolean prevDpadDown  = false;

    // ---- Fire window in RPM (from Constants) ----
    private static final double WINDOW_LOW  = Constants.Flywheel.FIRE_WINDOW_LOW;
    private static final double WINDOW_HIGH = Constants.Flywheel.FIRE_WINDOW_HIGH;

    @Override
    protected void initialize() {
        driveMode = false;

        // ---- Position: CLOSE ----
        posClose = new ButtonAction(
                () -> new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.CLOSE;
                            applyVelocity();
                        }),
                        new SetHoodAngle(ShootingPosition.CLOSE)
                ), commandRunner
        );

        // ---- Position: TOP (baseline) ----
        posTop = new ButtonAction(
                () -> new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.TOP;
                            applyVelocity();
                        }),
                        new SetHoodAngle(ShootingPosition.TOP)
                ), commandRunner
        );

        // ---- Position: FAR ----
        posFar = new ButtonAction(
                () -> new SeriesCommand(
                        new InstantCommand(() -> {
                            currentPosition = ShootingPosition.FAR;
                            applyVelocity();
                        }),
                        new SetHoodAngle(ShootingPosition.FAR)
                ), commandRunner
        );

        // ---- Fire: velocity-gated 3-artifact burst ----
        fireAction = new ButtonAction(
                () -> new ShootCurrent(() -> currentPosition), commandRunner
        );

        // ---- Stop flywheel ----
        stopAction = new ButtonAction(
                () -> new InstantCommand(() ->
                        OpModeBase.INSTANCE.setFlywheelVelocity(0)),
                commandRunner
        );

        // ---- Cycle step size ----
        cycleStep = new ButtonAction(
                () -> new InstantCommand(() ->
                        stepIndex = (stepIndex + 1) % STEPS.length),
                commandRunner
        );

        // ---- Spin up to TOP on init ----
        applyPIDF();
        currentPosition = ShootingPosition.TOP;
        OpModeBase.INSTANCE.hoodServo.setPosition(ShootingPosition.TOP.hoodPosition);
        applyVelocity();

        telemetry.addLine("Flywheel Tuner ready — spun up to TOP");
        telemetry.update();
    }

    @Override
    protected void runLoop() {
        double step = STEPS[stepIndex];

        // ---- D-pad edge detection for F and P ----
        boolean dpadLeft  = gamepad2.dpad_left;
        boolean dpadRight = gamepad2.dpad_right;
        boolean dpadUp    = gamepad2.dpad_up;
        boolean dpadDown  = gamepad2.dpad_down;

        if (dpadLeft  && !prevDpadLeft)  { liveF -= step; applyPIDF(); applyVelocity(); }
        if (dpadRight && !prevDpadRight) { liveF += step; applyPIDF(); applyVelocity(); }
        if (dpadUp    && !prevDpadUp)    { liveP += step; applyPIDF(); }
        if (dpadDown  && !prevDpadDown)  { liveP -= step; applyPIDF(); }

        prevDpadLeft  = dpadLeft;
        prevDpadRight = dpadRight;
        prevDpadUp    = dpadUp;
        prevDpadDown  = dpadDown;

        // ---- Position buttons ----
        posClose.update(gamepad2.a);
        posTop.update(gamepad2.y);
        posFar.update(gamepad2.b);

        // ---- Fire — only if in velocity window ----
        double actualRPM  = OpModeBase.INSTANCE.getFlywheelVelocity();
        double targetRPM  = currentPosition.targetVelocity;
        double errorRPM   = targetRPM - actualRPM;
        boolean inWindow  = errorRPM >= WINDOW_LOW && errorRPM <= WINDOW_HIGH;

        if (gamepad2.right_bumper && inWindow) {
            fireAction.update(true);
        } else {
            fireAction.update(false);
        }

        // ---- Stop flywheel ----
        stopAction.update(gamepad2.left_bumper);

        // ---- Cycle step ----
        cycleStep.update(gamepad2.x);
        // ---- Intake — manual load (no suppression during shoot sequence) ----
        double intakePower = gamepad2.left_stick_y;
        if (Math.abs(intakePower) > 0.1) {
            OpModeBase.INSTANCE.intakeMotor.setPower(intakePower);
        } else {
            OpModeBase.INSTANCE.intakeMotor.setPower(0);
        }

        // =====================================================
        // TELEMETRY
        // =====================================================
        telemetry.addLine("=== FLYWHEEL TUNER ===");
        telemetry.addData("Position",     currentPosition.name());
        telemetry.addData("Target RPM",   "%.0f", targetRPM);
        telemetry.addData("Actual RPM",   "%.0f", actualRPM);
        telemetry.addData("Error RPM",    "%.0f", errorRPM);
        telemetry.addData("In window",    inWindow ? "YES — ready to fire" : "NO  — waiting");
        telemetry.addLine("---");
        telemetry.addData("Live F",       "%.4f", liveF);
        telemetry.addData("Live P",       "%.4f", liveP);
        telemetry.addData("Step size",    "%.2f  (X to cycle)", step);
        telemetry.addLine("---");
        telemetry.addData("FL ticks/sec", "%.1f",
                OpModeBase.INSTANCE.flywheelLeft.getVelocity());
        telemetry.addData("FR ticks/sec", "%.1f",
                OpModeBase.INSTANCE.flywheelRight.getVelocity());
        telemetry.addLine("---");
        telemetry.addLine("Copy to Constants.Flywheel when happy:");
        telemetry.addData("  PIDF_F", "%.4f", liveF);
        telemetry.addData("  PIDF_P", "%.4f", liveP);
        telemetry.update();
    }

    // ---- Apply current liveF and liveP to both motors ----
    private void applyPIDF() {
        // Clamp to safe ranges — F and P can't go negative
        liveF = Math.max(0.0, liveF);
        liveP = Math.max(0.0, liveP);

        OpModeBase.INSTANCE.flywheelLeft.setVelocityPIDFCoefficients(
                liveP,
                Constants.Flywheel.PIDF_I,
                Constants.Flywheel.PIDF_D,
                liveF
        );
        OpModeBase.INSTANCE.flywheelRight.setVelocityPIDFCoefficients(
                liveP,
                Constants.Flywheel.PIDF_I,
                Constants.Flywheel.PIDF_D,
                liveF
        );
    }

    // ---- Command velocity for current position ----
    private void applyVelocity() {
        OpModeBase.INSTANCE.setFlywheelVelocity(currentPosition.targetVelocity);
    }
}