package gcsrobotics.opmode;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import gcsrobotics.commands.FollowPath;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.Command;
import gcsrobotics.vertices.CommandRunner;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.ParallelCommand;
import gcsrobotics.vertices.SeriesCommand;
import gcsrobotics.vertices.SleepCommand;

@Autonomous(name = "Far Zone Auto (Red)", group = "Hilda Auto")
public class FarZoneAutoRed extends OpModeBase {

    // ═══════════════════════════════════════════════════════════════════════
    // POSES — Red side mirror of Blue. Y = (144 - Blue Y) as starting point.
    // Verify every pose physically on the red side before competition!
    // ═══════════════════════════════════════════════════════════════════════

    private final Pose startPose = new Pose(
            12.0,               // TODO: X — same as blue
            36.0,               // TODO: Y — mirrored (144 - 108)
            Math.toRadians(0)   // TODO: Heading — confirm facing red goal
    );

    private final Pose shootPose = new Pose(
            24.0,               // TODO: X — same as blue
            36.0,               // TODO: Y — mirrored
            Math.toRadians(0)   // TODO: Heading — must face red goal
    );

    private final Pose tunnelEntryPose = new Pose(
            120.0,              // TODO: X — same wall side
            24.0,               // TODO: Y — mirrored (144 - 120)
            Math.toRadians(270) // TODO: Heading — flipped from blue's 90°
    );

    private final Pose tunnelFarPose = new Pose(
            120.0,              // TODO: X — same wall
            72.0,               // TODO: Y — tunnel runs toward field center
            Math.toRadians(270) // TODO: Heading — same as entry
    );

    private final Pose exitPose = new Pose(
            96.0,               // TODO: X — pulled back from wall
            72.0,               // TODO: Y — same as tunnel far end
            Math.toRadians(180) // TODO: Heading — facing back toward red goal for TeleOp
    );

    // ═══════════════════════════════════════════════════════════════════════
    // TIMING CONSTANTS — tune on hardware
    // ═══════════════════════════════════════════════════════════════════════
    private static final int SPINUP_MS     = 1500; // TODO: Tune spin-up time
    private static final int GATE_OPEN_MS  = 350;  // TODO: Tune gate open duration
    private static final int GATE_CLOSE_MS = 300;  // TODO: Tune gate close duration

    // Paths
    private PathChain pathToShoot;
    private PathChain pathToTunnelEntry;
    private PathChain pathThroughTunnel;
    private PathChain pathExitTunnel;

    private CommandRunner runner;

    // ─────────────────────────────────────────────────────────────────────
    @Override
    protected void initInternal() {
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
    protected void loopInternal() {
        if (runner == null) {
            runner = new CommandRunner(buildSequence());
            runner.start();
        }
        runner.update();

        telemetry.addData("Done", runner.isFinished());
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────
    // PATH CONSTRUCTION
    // ─────────────────────────────────────────────────────────────────────
    private void buildPaths() {

        pathToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        pathToTunnelEntry = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(72.0, 24.0, 0), // TODO: Control point
                        tunnelEntryPose
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), tunnelEntryPose.getHeading())
                .build();

        pathThroughTunnel = follower.pathBuilder()
                .addPath(new BezierLine(tunnelEntryPose, tunnelFarPose))
                .setConstantHeadingInterpolation(tunnelEntryPose.getHeading())
                .build();

        pathExitTunnel = follower.pathBuilder()
                .addPath(new BezierLine(tunnelFarPose, exitPose))
                .setLinearHeadingInterpolation(tunnelFarPose.getHeading(), exitPose.getHeading())
                .build();
    }

    // ─────────────────────────────────────────────────────────────────────
    // AUTONOMOUS SEQUENCE
    // ─────────────────────────────────────────────────────────────────────
    private Command buildSequence() {
        return new SeriesCommand(

                // ── PHASE 1: Spin up + drive to shoot pose simultaneously
                new ParallelCommand(
                        spinUpShooter(),
                        new FollowPath(pathToShoot)
                ),

                // ── PHASE 2: Brief stabilization pause
                new SleepCommand(200),

                // ── PHASE 3: Fire all 3 preloads
                fireShot(),
                fireShot(),
                fireShot(),

                // ── PHASE 4: Spin down shooter
                new InstantCommand(() -> {
                    setFlywheelVelocity(0);
                    gateServo.setPosition(Constants.Gate.CLOSE_POSITION);
                    hoodServo.setPosition(Constants.Hood.CLOSE);
                }),

                // ── PHASE 5: Drive to tunnel, start intake on the way
                new ParallelCommand(
                        new SeriesCommand(
                                new SleepCommand(500),
                                new InstantCommand(() ->
                                        intakeMotor.setPower(Constants.Intake.FORWARD_POWER))
                        ),
                        new FollowPath(pathToTunnelEntry)
                ),

                // ── PHASE 6: Drive through tunnel with intake running
                new FollowPath(pathThroughTunnel),

                // ── PHASE 7: Stop intake, exit tunnel — TeleOp takes over
                new InstantCommand(() -> intakeMotor.setPower(0)),
                new FollowPath(pathExitTunnel)
        );
    }

    // ─────────────────────────────────────────────────────────────────────
    // HELPER COMMANDS
    // ─────────────────────────────────────────────────────────────────────

    private Command spinUpShooter() {
        return new SeriesCommand(
                new InstantCommand(() -> {
                    hoodServo.setPosition(Constants.Hood.FAR);
                    setFlywheelVelocity(Constants.Flywheel.VELOCITY_FAR);
                }),
                new SleepCommand(SPINUP_MS)
        );
    }

    private Command fireShot() {
        return new SeriesCommand(
                new InstantCommand(() ->
                        gateServo.setPosition(Constants.Gate.OPEN_POSITION)),
                new SleepCommand(GATE_OPEN_MS),
                new InstantCommand(() ->
                        gateServo.setPosition(Constants.Gate.CLOSE_POSITION)),
                new SleepCommand(GATE_CLOSE_MS)
        );
    }
}