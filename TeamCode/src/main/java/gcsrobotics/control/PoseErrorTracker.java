package gcsrobotics.control;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

// ═══════════════════════════════════════════════════════════════════════════
// PoseErrorTracker
//
// Records pose error (actual vs target) at named checkpoints during
// autonomous. Call record() at each checkpoint, then display() every
// loop to show all accumulated errors on telemetry.
// ═══════════════════════════════════════════════════════════════════════════
public class PoseErrorTracker {

    // ─────────────────────────────────────────────────────────────────────
    // Internal data class for a single checkpoint
    // ─────────────────────────────────────────────────────────────────────
    private static class Checkpoint {
        final String name;
        final double xError;
        final double yError;
        final double headingErrorDeg;

        Checkpoint(String name, Pose actual, Pose target) {
            this.name            = name;
            this.xError          = actual.getX()       - target.getX();
            this.yError          = actual.getY()       - target.getY();
            this.headingErrorDeg = Math.toDegrees(actual.getHeading())
                    - Math.toDegrees(target.getHeading());
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Storage
    // ─────────────────────────────────────────────────────────────────────
    private final List<Checkpoint> checkpoints = new ArrayList<>();

    // ─────────────────────────────────────────────────────────────────────
    // record()
    // Call immediately after a path finishes at a named pose.
    // ─────────────────────────────────────────────────────────────────────
    public void record(String name, Pose actual, Pose target) {
        checkpoints.add(new Checkpoint(name, actual, target));
    }

    // ─────────────────────────────────────────────────────────────────────
    // display()
    // Call every loop tick inside runLoop() to show all checkpoints.
    // ─────────────────────────────────────────────────────────────────────
    public void display(Telemetry telemetry) {
        if (checkpoints.isEmpty()) {
            telemetry.addLine("PoseErrorTracker: no checkpoints yet");
            return;
        }
        for (Checkpoint cp : checkpoints) {
            telemetry.addData(cp.name + " xErr",    "%.2f", cp.xError);
            telemetry.addData(cp.name + " yErr",    "%.2f", cp.yError);
            telemetry.addData(cp.name + " hdgErr",  "%.1f°", cp.headingErrorDeg);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // clear()
    // Optional — resets all recorded checkpoints.
    // ─────────────────────────────────────────────────────────────────────
    public void clear() {
        checkpoints.clear();
    }
}