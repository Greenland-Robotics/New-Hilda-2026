package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
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
 * ║  ABOUT THE SECRET TUNNEL:                                               ║
 * ║  The SECRET TUNNEL ZONE is ~46.5 in. long x ~6.125 in. wide,          ║
 * ║  located along the field perimeter between the opposing alliance's      ║
 * ║  GOAL and LOADING ZONE. Overflow artifacts land here from the ramp.    ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  ⚠  ALL POSES ARE PLACEHOLDERS — measure with Localization Test first! ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 */
@Autonomous(name = "Far Zone Auto (Blue)", group = "Hilda Auto")
public class FarZoneAutoBlue extends OpModeBase {

    // ═══════════════════════════════════════════════════════════════════════
    // POSES  —  measure all of these with the Localization Test OpMode
    // ═══════════════════════════════════════════════════════════════════════

    // Far launch zone start tile (audience side triangular zone)
    // TODO: Run Localization Test, drive to start tile, record X/Y/heading
    private final Pose startPose = new Pose(
            12.0,               // TODO: X inches from back wall to robot center
            108.0,              // TODO: Y inches from right wall to robot center
            Math.toRadians(0)   // TODO: Heading — face toward goal
    );

    // Where Hilda stops to shoot preloads
    // TODO: Test shooting accuracy from far zone distance
    private final Pose shootPose = new Pose(
            24.0,               // TODO: X — adjust for shooting distance to goal
            108.0,              // TODO: Y — stay on far side of field
            Math.toRadians(0)   // TODO: Heading — must face goal
    );

    // Entry point of the SECRET TUNNEL ZONE (~6.125 in. wide — approach carefully)
    // TODO: Measure tunnel entry with Localization Test
    private final Pose tunnelEntryPose = new Pose(
            120.0,              // TODO: X — near loading zone wall
            120.0,              // TODO: Y — aligned with tunnel opening
            Math.toRadians(90)  // TODO: Heading — facing into tunnel along its length
    );

    // Far end of SECRET TUNNEL ZONE (tunnel is ~46.5 in. long total)
    // TODO: Measure tunnel far end
    private final Pose tunnelFarPose = new Pose(
            120.0,              // TODO: X — same wall, deeper into tunnel
            72.0,               // TODO: Y — ~46.5 in. through tunnel from entry
            Math.toRadians(90)  // TODO: Heading — same as entry (straight through)
    );

    // Exit pose — clear of tunnel, ready for TeleOp
    // TODO: Confirm this is fully outside the tunnel zone boundary
    private final Pose exitPose = new Pose(
            96.0,               // TODO: X — pulled back from tunnel wall
            72.0,               // TODO: Y — same Y as tunnel far end
            Math.toRadians(180) // TODO: Heading — facing back toward goal for TeleOp
    );

    // ═══════════════════════════════════════════════════════════════════════
    // SHOOTER CONSTANTS  —  tune at actual far-zone shooting distance
    // ═══════════════════════════════════════════════════════════════════════
    private static final double SHOOTER_POWER = 0.85;  // TODO: Tune for far zone distance
    private static final double HOOD_FAR_ZONE = 0.50;  // TODO: Tune servo angle for far shot
    private static final int    SPINUP_MS     = 1500;  // TODO: Tune — time to reach full speed
    private static final int    GATE_OPEN_MS  = 350;   // TODO: Tune — time to release one artifact
    private static final int    GATE_CLOSE_MS = 300;   // TODO: Tune — reset between shots
    private static final double GATE_FIRE_POS = 0.0;   // TODO: Pull from Constants.Gate.OPEN
    private static final double GATE_HOLD_POS = 0.5;   // TODO: Pull from Constants.Gate.CLOSED

    // ═══════════════════════════════════════════════════════════════════════
    // INTAKE CONSTANTS
    // ═══════════════════════════════════════════════════════════════════════
    private static final double INTAKE_POWER = 1.0;  // Full power for tunnel sweep

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

        telemetry.addLine("=== FAR ZONE AUTO — BLUE ===");
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
    // PATH CONSTRUCTION
    // ─────────────────────────────────────────────────────────────────────
    private void buildPaths() {

        // Start → Shoot: straight line
        pathToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(shootPose)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Shoot → Tunnel Entry: curve around any field obstacles
        // TODO: Adjust control point based on actual field layout
        pathToTunnelEntry = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(shootPose),
                        new Point(72.0, 120.0, Point.CARTESIAN), // TODO: Control point
                        new Point(tunnelEntryPose)
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), tunnelEntryPose.getHeading())
                .build();

        // Through tunnel: slow straight drive with intake running
        // The tunnel is only ~6.125 in. wide — approach carefully!
        pathThroughTunnel = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(tunnelEntryPose),
                        new Point(tunnelFarPose)
                ))
                .setConstantHeadingInterpolation(tunnelEntryPose.getHeading())
                .build();

        // Exit tunnel
        pathExitTunnel = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(tunnelFarPose),
                        new Point(exitPose)
                ))
                .setLinearHeadingInterpolation(tunnelFarPose.getHeading(), exitPose.getHeading())
                .build();
    }

    // ─────────────────────────────────────────────────────────────────────
    // AUTONOMOUS SEQUENCE
    //
    // Estimated timeline:
    //   0.0s   Spin up + drive to shoot pose (parallel)
    //   ~2.0s  Fire shot 1
    //   ~2.8s  Fire shot 2
    //   ~3.6s  Fire shot 3
    //   ~4.5s  Spin down, drive to tunnel
    //   ~8.0s  Enter tunnel, intake sweeping
    //   ~11s   Clear tunnel, intake off
    //   ~13s   Exit pose — TeleOp takes over
    // ─────────────────────────────────────────────────────────────────────
    private Command buildSequence() {
        return new SeriesCommand(

                // ── PHASE 1: Spin up shooter + drive to shoot pose simultaneously
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
    // HELPER COMMANDS
    // ─────────────────────────────────────────────────────────────────────

    /** Spin up both flywheels and set hood for far zone shot. */
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

    /** Open gate to fire one artifact, then close and reset. */
    private Command fireShot() {
        return new SeriesCommand(
                new InstantCommand(() -> gateServo.setPosition(GATE_FIRE_POS)),
                new SleepCommand(GATE_OPEN_MS),
                new InstantCommand(() -> gateServo.setPosition(GATE_HOLD_POS)),
                new SleepCommand(GATE_CLOSE_MS)
        );
    }
}
