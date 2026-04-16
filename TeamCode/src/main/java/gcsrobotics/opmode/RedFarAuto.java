package gcsrobotics.opmode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import gcsrobotics.control.PoseStorage;
import gcsrobotics.commands.FollowPath;
import gcsrobotics.commands.ShootCurrent;
import gcsrobotics.commands.ShootingPosition;
import gcsrobotics.commands.StartIntake;
import gcsrobotics.control.AutoBase;
import gcsrobotics.control.PoseErrorTracker;
import gcsrobotics.control.OpModeBase;
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.vertices.CommandRunner;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.SeriesCommand;
import gcsrobotics.vertices.SleepCommand;


// ═══════════════════════════════════════════════════════════════════════════
// RedFarAuto — Far Zone Autonomous (Red Alliance)
//
// FLYWHEEL: runs at VELOCITY_FAR throughout — no idle between cycles
//
// SHOOT POSE HEADINGS (field test adjustments):
//   Shot 1 (Preload)  → SHOOT_POSE          62° (2° LEFT from 64°)
//   Shot 2 (Corner1)  → SHOOT_POSE_RETURN   64° (2° RIGHT from 62°)
//   Shot 3 (Spike)    → SHOOT_POSE_RETURN_2 61° (NO CHANGE — already good)
//   Shot 4 (Corner2)  → SHOOT_POSE_RETURN_3 62° (2° RIGHT from 60°)
//
// CHASSIS SPEED CHANGES:
//   SpikeMarkCycle:
//     - Approach curve       → NORMAL_SPEED
//     - Collect sweep        → INTAKE_SPEED  (slow)
//     - Return to shoot      → SPIKE_RETURN_SPEED (0.75 — slower for accuracy)
//   CornerCycle (x2):
//     - Approach             → NORMAL_SPEED
//     - Bump forward/back    → INTAKE_SPEED  (slow)
//     - Return               → NORMAL_SPEED
// ═══════════════════════════════════════════════════════════════════════════


@Autonomous(name = "RedFarAuto")
public class RedFarAuto extends AutoBase {

    // ─────────────────────────────────────────────────────────────────────
    // TIMING CONSTANTS
    // ─────────────────────────────────────────────────────────────────────
    private static final int SPINUP_MS         = 1600; // first shot only
    private static final int STABILIZE_MS      = 200;
    private static final int BUMP_PAUSE_MS     = 500;
    private static final int HEADING_SETTLE_MS = 300;

    // ─────────────────────────────────────────────────────────────────────
    // DRIVE SPEED CONSTANTS
    // ─────────────────────────────────────────────────────────────────────
    private static final double NORMAL_SPEED       = 1.0;
    private static final double INTAKE_SPEED       = 0.8;
    private static final double SPIKE_RETURN_SPEED = 0.75;

    // ─────────────────────────────────────────────────────────────────────
    // START POSE
    // ─────────────────────────────────────────────────────────────────────
    private static final Pose START_POSE = new Pose(80, 9, Math.toRadians(90));

    // ─────────────────────────────────────────────────────────────────────
    // SHOOT POSES — heading adjustments per field test results
    // ─────────────────────────────────────────────────────────────────────
    static final Pose SHOOT_POSE          = new Pose(85.000, 14.000, Math.toRadians(67)); // Shot 1 — preload (2° LEFT)
    static final Pose SHOOT_POSE_RETURN   = new Pose(85.000, 14.000, Math.toRadians(57)); // Shot 2 — Corner1 (2° RIGHT)
    static final Pose SHOOT_POSE_RETURN_2 = new Pose(85.000, 14.000, Math.toRadians(63)); // Shot 3 — Spike (NO CHANGE)
    static final Pose SHOOT_POSE_RETURN_3 = new Pose(85.000, 14.000, Math.toRadians(59)); // Shot 4 — Corner2 (2° RIGHT)

    Paths paths;
    PoseErrorTracker tracker = new PoseErrorTracker();

    // ─────────────────────────────────────────────────────────────────────
    // INITIALIZE
    // ─────────────────────────────────────────────────────────────────────
    @Override
    protected void initialize() {
        follower.setStartingPose(START_POSE);
        paths = new Paths(follower);
        gateServo.setPosition(Constants.Gate.CLOSE_POSITION);
        hoodServo.setPosition(Constants.Hood.FAR);
    }

    // ─────────────────────────────────────────────────────────────────────
    // BUILD COMMANDS
    // ─────────────────────────────────────────────────────────────────────
    @Override
    protected void buildCommands() {
        commandRunner = new CommandRunner(
                new SeriesCommand(

                        // ── STEP 1: Spin up flywheel + set hood — stays on all auto
                        new InstantCommand(() -> {
                            setFlywheelVelocity(Constants.Flywheel.VELOCITY_FAR);
                            hoodServo.setPosition(Constants.Hood.FAR);
                            follower.setMaxPower(NORMAL_SPEED);
                        }),

                        // ── STEP 2: Drive to shoot pose
                        new FollowPath(paths.PreloadShoot),

                        // ── STEP 3: Wait for flywheel to reach speed (first shot only)
                        new SleepCommand(SPINUP_MS),

                        // ── STEP 4: Shoot preloads
                        new ShootCurrent(() -> ShootingPosition.FAR),
                        new InstantCommand(() -> tracker.record("PRELOAD_SHOOT", follower.getPose(), SHOOT_POSE)),

                        // ── STEP 5: Start intake
                        new StartIntake(),

                        // ══════════════════════════════════════════════
                        // CORNER CYCLE 1
                        // ══════════════════════════════════════════════

                        new FollowPath(paths.CornerApproach),
                        new InstantCommand(() -> tracker.record("CORNER_APPROACH", follower.getPose(), Paths.CORNER_APPROACH_POSE)),

                        new InstantCommand(() -> follower.setMaxPower(INTAKE_SPEED)),
                        new FollowPath(paths.CornerBumpBack),
                        new InstantCommand(() -> tracker.record("CORNER_BUMP_BACK", follower.getPose(), Paths.CORNER_BUMP_BACK_POSE)),
                        new FollowPath(paths.CornerBumpForward),
                        new InstantCommand(() -> tracker.record("CORNER_BUMP_FWD", follower.getPose(), Paths.CORNER_BUMP_FWD_POSE)),
                        new FollowPath(paths.CornerAdjust),
                        new InstantCommand(() -> tracker.record("CORNER_ADJUST", follower.getPose(), Paths.CORNER_ADJUST_POSE)),

                        new SleepCommand(BUMP_PAUSE_MS),

                        new InstantCommand(() -> follower.setMaxPower(NORMAL_SPEED)),
                        new FollowPath(paths.CornerReturn1),
                        new SleepCommand(STABILIZE_MS),
                        new ShootCurrent(() -> ShootingPosition.FAR),
                        new InstantCommand(() -> tracker.record("CORNER1_SHOOT", follower.getPose(), SHOOT_POSE_RETURN)),
                        new StartIntake(),

                        // ══════════════════════════════════════════════
                        // SPIKE MARK CYCLE
                        // ══════════════════════════════════════════════

                        new SleepCommand(HEADING_SETTLE_MS),

                        new InstantCommand(() -> follower.setMaxPower(NORMAL_SPEED)),
                        new FollowPath(paths.SpikeApproach),
                        new InstantCommand(() -> tracker.record("SPIKE_APPROACH", follower.getPose(), Paths.SPIKE_APPROACH_POSE)),

                        new InstantCommand(() -> follower.setMaxPower(INTAKE_SPEED)),
                        new FollowPath(paths.SpikeSweep),
                        new InstantCommand(() -> tracker.record("SPIKE_SWEEP_END", follower.getPose(), Paths.SPIKE_SWEEP_END_POSE)),

                        new InstantCommand(() -> follower.setMaxPower(SPIKE_RETURN_SPEED)),
                        new FollowPath(paths.SpikeReturn),
                        new SleepCommand(STABILIZE_MS),
                        new ShootCurrent(() -> ShootingPosition.FAR),
                        new InstantCommand(() -> tracker.record("SPIKE_SHOOT", follower.getPose(), SHOOT_POSE_RETURN_2)),
                        new StartIntake(),

                        // ══════════════════════════════════════════════
                        // CORNER CYCLE 2
                        // ══════════════════════════════════════════════

                        new InstantCommand(() -> follower.setMaxPower(NORMAL_SPEED)),
                        new FollowPath(paths.CornerApproach2),
                        new InstantCommand(() -> tracker.record("CORNER2_APPROACH", follower.getPose(), Paths.CORNER_APPROACH_POSE)),

                        new InstantCommand(() -> follower.setMaxPower(INTAKE_SPEED)),
                        new FollowPath(paths.CornerAdjust),
                        new InstantCommand(() -> tracker.record("CORNER2_ADJUST", follower.getPose(), Paths.CORNER_ADJUST_POSE)),

                        new SleepCommand(BUMP_PAUSE_MS),

                        new InstantCommand(() -> follower.setMaxPower(NORMAL_SPEED)),
                        new FollowPath(paths.CornerReturn2),
                        new SleepCommand(STABILIZE_MS),
                        new ShootCurrent(() -> ShootingPosition.FAR),
                        new InstantCommand(() -> tracker.record("CORNER2_SHOOT", follower.getPose(), SHOOT_POSE_RETURN_3)),

                        // ── FINAL: Stop intake + park
                        new InstantCommand(() -> {
                            intakeMotor.setPower(0);
                            setFlywheelVelocity(0);
                            gateServo.setPosition(Constants.Gate.CLOSE_POSITION);
                            follower.setMaxPower(NORMAL_SPEED);
                        }),
                        new FollowPath(paths.Park),
                        new InstantCommand(() -> tracker.record("PARK_END", follower.getPose(), Paths.PARK_POSE)),
                        new InstantCommand(() -> PoseStorage.currentPose = follower.getPose())
                )
        );
    }

    // ─────────────────────────────────────────────────────────────────────
    // RUN LOOP
    // ─────────────────────────────────────────────────────────────────────
    @Override
    protected void runLoop() {
        updateFlywheel();
        telemetry.addData("Pedro X",      "%.2f", follower.getPose().getX());
        telemetry.addData("Pedro Y",      "%.2f", follower.getPose().getY());
        telemetry.addData("Heading",      "%.1f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Flywheel RPM", "%.0f", getFlywheelVelocity());
        telemetry.addData("Auto Done",    commandRunner.isFinished());
        telemetry.addLine("---");
        tracker.display(telemetry);
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────
    // PATHS
    // ─────────────────────────────────────────────────────────────────────
    static class Paths {

        // ── Endpoint pose constants ──
        static final Pose CORNER_APPROACH_POSE  = new Pose(130.000,  8.700, Math.toRadians(0));
        static final Pose CORNER_BUMP_BACK_POSE = new Pose(126,  8.7, Math.toRadians(0));
        static final Pose CORNER_BUMP_FWD_POSE  = new Pose(130.000,  8.600, Math.toRadians(0));
        static final Pose CORNER_ADJUST_POSE    = new Pose(122.000, 12.600, Math.toRadians(0));
        static final Pose SPIKE_APPROACH_POSE   = new Pose( 99.952, 36, Math.toRadians(0));
        static final Pose SPIKE_SWEEP_END_POSE  = new Pose(127.433, 36, Math.toRadians(0));
        static final Pose PARK_POSE             = new Pose(112.749,  8.708, Math.toRadians(0));

        // ── PathChain declarations ──
        public PathChain PreloadShoot;
        public PathChain SpikeApproach;
        public PathChain SpikeSweep;
        public PathChain SpikeReturn;
        public PathChain CornerApproach;   // departs from SHOOT_POSE_RETURN (64°)
        public PathChain CornerApproach2;  // departs from SHOOT_POSE_RETURN_2 (61°)
        public PathChain CornerBumpForward;
        public PathChain CornerBumpBack;
        public PathChain CornerAdjust;
        public PathChain CornerReturn1;    // arrives at SHOOT_POSE_RETURN (64°)
        public PathChain CornerReturn2;    // arrives at SHOOT_POSE_RETURN_3 (62°)
        public PathChain Park;

        public Paths(Follower follower) {

            // ── Preload — targets SHOOT_POSE at 62° ──
            PreloadShoot = follower.pathBuilder()
                    .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            // ── Corner Cycle 1 — departs from SHOOT_POSE_RETURN (64°) ──
            CornerApproach = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE_RETURN, CORNER_APPROACH_POSE))
                    .setLinearHeadingInterpolation(SHOOT_POSE_RETURN.getHeading(), CORNER_APPROACH_POSE.getHeading())
                    .build();

            CornerBumpBack = follower.pathBuilder()
                    .addPath(new BezierLine(CORNER_APPROACH_POSE, CORNER_BUMP_BACK_POSE))
                    .setLinearHeadingInterpolation(CORNER_APPROACH_POSE.getHeading(), CORNER_BUMP_BACK_POSE.getHeading())
                    .build();

            CornerBumpForward = follower.pathBuilder()
                    .addPath(new BezierLine(CORNER_BUMP_BACK_POSE, CORNER_BUMP_FWD_POSE))
                    .setLinearHeadingInterpolation(CORNER_BUMP_BACK_POSE.getHeading(), CORNER_BUMP_FWD_POSE.getHeading())
                    .build();

            CornerAdjust = follower.pathBuilder()
                    .addPath(new BezierLine(CORNER_BUMP_FWD_POSE, CORNER_ADJUST_POSE))
                    .setLinearHeadingInterpolation(CORNER_BUMP_FWD_POSE.getHeading(), CORNER_ADJUST_POSE.getHeading())
                    .build();

            // ── Corner Return 1 — arrives at SHOOT_POSE_RETURN (64°) ──
            CornerReturn1 = follower.pathBuilder()
                    .addPath(new BezierLine(CORNER_ADJUST_POSE, SHOOT_POSE_RETURN))
                    .setLinearHeadingInterpolation(CORNER_ADJUST_POSE.getHeading(), SHOOT_POSE_RETURN.getHeading())
                    .build();

            // ── Spike Mark Cycle — departs from SHOOT_POSE_RETURN (64°) ──
            SpikeApproach = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            SHOOT_POSE_RETURN,
                            new Pose(90.434, 36.425),
                            SPIKE_APPROACH_POSE))
                    .setLinearHeadingInterpolation(SHOOT_POSE_RETURN.getHeading(), SPIKE_APPROACH_POSE.getHeading())
                    .build();

            SpikeSweep = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE_APPROACH_POSE, SPIKE_SWEEP_END_POSE))
                    .setLinearHeadingInterpolation(SPIKE_APPROACH_POSE.getHeading(), SPIKE_SWEEP_END_POSE.getHeading())
                    .build();

            // ── Spike Return — arrives at SHOOT_POSE_RETURN_2 (61°) ──
            SpikeReturn = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE_SWEEP_END_POSE, SHOOT_POSE_RETURN_2))
                    .setLinearHeadingInterpolation(SPIKE_SWEEP_END_POSE.getHeading(), SHOOT_POSE_RETURN_2.getHeading())
                    .build();

            // ── Corner Cycle 2 — departs from SHOOT_POSE_RETURN_2 (61°) ──
            CornerApproach2 = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE_RETURN_2, CORNER_APPROACH_POSE))
                    .setLinearHeadingInterpolation(SHOOT_POSE_RETURN_2.getHeading(), CORNER_APPROACH_POSE.getHeading())
                    .build();

            // ── Corner Return 2 — arrives at SHOOT_POSE_RETURN_3 (62°) ──
            CornerReturn2 = follower.pathBuilder()
                    .addPath(new BezierLine(CORNER_ADJUST_POSE, SHOOT_POSE_RETURN_3))
                    .setLinearHeadingInterpolation(CORNER_ADJUST_POSE.getHeading(), SHOOT_POSE_RETURN_3.getHeading())
                    .build();

            // ── Park — departs from SHOOT_POSE_RETURN_3 (62°) ──
            Park = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE_RETURN_3, PARK_POSE))
                    .setLinearHeadingInterpolation(SHOOT_POSE_RETURN_3.getHeading(), PARK_POSE.getHeading())
                    .build();
        }
    }
}


