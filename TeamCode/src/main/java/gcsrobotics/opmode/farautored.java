package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import gcsrobotics.vertices.Command;
import gcsrobotics.vertices.CommandRunner;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.ParallelCommand;
import gcsrobotics.vertices.SeriesCommand;
import gcsrobotics.vertices.SleepCommand;

import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@Autonomous(name = "Far Zone Auto (Red)", group = "Hilda Auto")
public class FarZoneAutoRed extends OpModeBase {

    // ═══════════════════════════════════════════════════════════════════════
    // POSES — Red side mirror of Blue. Y = (144 - Blue Y) as starting point.
    // Verify every pose physically on the red side before competition!
    // ═══════════════════════════════════════════════════════════════════════

    // Blue Y=108 → Red Y=36
    // TODO: Confirm with Localization Test on red far launch zone tile
    private final Pose startPose = new Pose(
            12.0,               // TODO: X — same as blue
            36.0,               // TODO: Y — mirrored (144 - 108)
            Math.toRadians(0)   // TODO: Heading — confirm facing red goal
    );

    // Blue Y=108 → Red Y=36
    // TODO: Test shooting accuracy from red far zone position
    private final Pose shootPose = new Pose(
            24.0,               // TODO: X — same as blue
            36.0,               // TODO: Y — mirrored
            Math.toRadians(0)   // TODO: Heading — must face red goal
    );

    // Blue Y=120 → Red Y=24
    // TODO: Measure red tunnel entry — only ~6.125 in. wide, approach carefully
    private final Pose tunnelEntryPose = new Pose(
            120.0,              // TODO: X — same wall side
            24.0,               // TODO: Y — mirrored (144 - 120)
            Math.toRadians(270) // TODO: Heading — flipped from blue's 90°
    );

    // Blue Y=72 → Red Y=72 (center of field — same either side)
    // TODO: Measure red tunnel far end
    private final Pose tunnelFarPose = new Pose(
            120.0,              // TODO: X — same wall
            72.0,               // TODO: Y — tunnel runs toward field center
            Math.toRadians(270) // TODO: Heading — same as entry (straight through)
    );

    // TODO: Confirm fully outside tunnel zone boundary on red side
    private final Pose exitPose = new Pose(
            96.0,               // TODO: X — pulled back from wall
            72.0,               // TODO: Y — same as tunnel far end
            Math.toRadians(180) // TODO: Heading — facing back toward red goal for TeleOp
    );

    // ═══════════════════════════════════════════════════════════════════════
    // SHOOTER CONSTANTS — identical to blue (same robot, same shooter)
    // ═══════════════════════════════════════════════════════════════════════
    private static final double SHOOTER_POWER = 0.85;  // TODO: Tune for far zone distance
    private static final double HOOD_FAR_ZONE = 0.50;  // TODO: Tune servo angle for far shot
    private static final int    SPINUP_MS     = 1500;  // TODO: Tune spin-up wait
    private static final int    GATE_OPEN_MS  = 350;   // TODO: Tune gate open time
    private static final int    GATE_CLOSE_MS = 300;   // TODO: Tune gate reset time
    private static final double GATE_FIRE_POS = 0.0;   // TODO: Pull from Constants.Gate.OPEN
    private static final double GATE_HOLD_POS = 0.5;   // TODO: Pull from Constants.Gate.CLOSED
    private static final double INTAKE_POWER  = 1.0;

    // Paths
    private PathChain pathToShoot;
    private PathChain pathToTunnelEntry;
    private PathChain pathThroughTunnel;
    private PathChain pathExitTunnel;

    private CommandRunner runner;

    // ─────────────────────────────────────────────────────────────────────
    @Override
    public void inInit() {
        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("=== FAR ZONE AUTO — RED ===");
        telemetry.addLine("Sequence: Shoot (x3) → Secret Tunnel → Exit");
        telemetry.addLine("⚠ PLACEHOLDER COORDINATES — measure before running!");
        telemetry.addData("Start Pose", "(%.1f, %.1f) @ %.0f°",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────
    @Override
    public void runLoop() {
        if (runner == null) {
            runner = new CommandRunner(buildSequence());
        }
        runner.run();

        follower.telemetryDebug(telemetry);
        telemetry.addData("Done", runner.isFinished());
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────
    private void buildPaths() {

        pathToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(shootPose)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Control point mirrored from blue (Y = 144 - 120 = 24.0)
        // TODO: Adjust based on actual red side field layout
        pathToTunnelEntry = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(shootPose),
                        new Point(72.0, 24.0, Point.CARTESIAN), // TODO: Control point
                        new Point(tunnelEntryPose)
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), tunnelEntryPose.getHeading())
                .build();

        pathThroughTunnel = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(tunnelEntryPose),
                        new Point(tunnelFarPose)
                ))
                .setConstantHeadingInterpolation(tunnelEntryPose.getHeading())
                .build();

        pathExitTunnel = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(tunnelFarPose),
                        new Point(exitPose)
                ))
                .setLinearHeadingInterpolation(tunnelFarPose.getHeading(), exitPose.getHeading())
                .build();
    }

    // ─────────────────────────────────────────────────────────────────────
    private Command buildSequence() {
        return new SeriesCommand(

                // ── PHASE 1: Spin up + drive to shoot pose simultaneously
                new ParallelCommand(
                        spinUpShooter(),
                        new FollowPath(follower, pathToShoot)
                ),

                // ── PHASE 2: Brief stabilization pause
                new SleepCommand(200),

                // ── PHASE 3: Fire all 3 preloads
                fireShot(),
                fireShot(),
                fireShot(),

                // ── PHASE 4: Spin down shooter
                new InstantCommand(() -> {
                    shooter1.setPower(0);
                    shooter2.setPower(0);
                    gateServo.setPosition(GATE_HOLD_POS);
                    hoodServo.setPosition(0.0); // TODO: Set travel position
                }),

                // ── PHASE 5: Drive to tunnel, start intake on the way
                new ParallelCommand(
                        new SeriesCommand(
                                new SleepCommand(500),
                                new InstantCommand(() -> intakeMotor.setPower(INTAKE_POWER))
                        ),
                        new FollowPath(follower, pathToTunnelEntry)
                ),

                // ── PHASE 6: Drive through tunnel with intake running
                new FollowPath(follower, pathThroughTunnel),

                // ── PHASE 7: Stop intake, exit tunnel — TeleOp takes over
                new InstantCommand(() -> intakeMotor.setPower(0)),
                new FollowPath(follower, pathExitTunnel)
        );
    }

    // ─────────────────────────────────────────────────────────────────────
    private Command spinUpShooter() {
        return new SeriesCommand(
                new InstantCommand(() -> {
                    hoodServo.setPosition(HOOD_FAR_ZONE);
                    shooter1.setPower(SHOOTER_POWER);
                    shooter2.setPower(SHOOTER_POWER);
                }),
                new SleepCommand(SPINUP_MS)
        );
    }

    private Command fireShot() {
        return new SeriesCommand(
                new InstantCommand(() -> gateServo.setPosition(GATE_FIRE_POS)),
                new SleepCommand(GATE_OPEN_MS),
                new InstantCommand(() -> gateServo.setPosition(GATE_HOLD_POS)),
                new SleepCommand(GATE_CLOSE_MS)
        );
    }
}