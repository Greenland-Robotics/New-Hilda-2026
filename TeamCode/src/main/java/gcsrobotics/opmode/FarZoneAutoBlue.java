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

/**
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║         FAR ZONE AUTONOMOUS — BLUE ALLIANCE                            ║
 * ║         GCS Robotics / Hilda the Builda 2026 / DECODE Season           ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  SEQUENCE:                                                              ║
 * ║    1. Spin up shooter + drive to shoot pose (parallel)                  ║
 * ║    2. Fire 3 preloaded artifacts into goal                              ║
 * ║    3. Drive to SECRET TUNNEL ZONE to collect overflow artifacts         ║
 * ║    4. Run intake through the tunnel                                     ║
 * ║    5. Exit tunnel — stop and hand off to TeleOp                        ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  ⚠  ALL POSES ARE PLACEHOLDERS — measure with Localization Test first! ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */
@Autonomous(name = "Far Zone Auto (Blue)", group = "Hilda Auto")
public class FarZoneAutoBlue extends OpModeBase {

    // ═══════════════════════════════════════════════════════════════════════
    // POSES — measure all of these with the Localization Test OpMode
    // ═══════════════════════════════════════════════════════════════════════

    private final Pose startPose = new Pose(
            12.0,               // TODO: X inches from back wall to robot center
            108.0,              // TODO: Y inches from right wall to robot center
            Math.toRadians(0)   // TODO: Heading — face toward goal
    );

    private final Pose shootPose = new Pose(
            24.0,               // TODO: X — adjust for shooting distance to goal
            108.0,              // TODO: Y — stay on far side of field
            Math.toRadians(0)   // TODO: Heading — must face goal
    );

    private final Pose tunnelEntryPose = new Pose(
            120.0,              // TODO: X — near loading zone wall
            120.0,              // TODO: Y — aligned with tunnel opening
            Math.toRadians(90)  // TODO: Heading — facing into tunnel
    );

    private final Pose tunnelFarPose = new Pose(
            120.0,              // TODO: X — same wall, deeper into tunnel
            72.0,               // TODO: Y — ~46.5 in. through tunnel from entry
            Math.toRadians(90)  // TODO: Heading — same as entry
    );

    private final Pose exitPose = new Pose(
            96.0,               // TODO: X — pulled back from tunnel wall
            72.0,               // TODO: Y — same Y as tunnel far end
            Math.toRadians(180) // TODO: Heading — facing back toward goal for TeleOp
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

        telemetry.addLine("=== FAR ZONE AUTO — BLUE ===");
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

        // Start → Shoot: straight line
        pathToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Shoot → Tunnel Entry: curve around field obstacles
        pathToTunnelEntry = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(72.0, 120.0, 0), // TODO: Control point
                        tunnelEntryPose
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), tunnelEntryPose.getHeading())
                .build();

        // Through tunnel: straight drive with intake running
        pathThroughTunnel = follower.pathBuilder()
                .addPath(new BezierLine(tunnelEntryPose, tunnelFarPose))
                .setConstantHeadingInterpolation(tunnelEntryPose.getHeading())
                .build();

        // Exit tunnel
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

                // ── PHASE 1: Spin up shooter + drive to shoot pose simultaneously
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