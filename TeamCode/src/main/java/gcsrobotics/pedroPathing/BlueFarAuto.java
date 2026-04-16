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
import gcsrobotics.pedroPathing.Constants;
import gcsrobotics.pedroPathing.FieldMirror;
import gcsrobotics.vertices.CommandRunner;
import gcsrobotics.vertices.InstantCommand;
import gcsrobotics.vertices.SeriesCommand;
import gcsrobotics.vertices.SleepCommand;

// ═══════════════════════════════════════════════════════════════════════════
// BlueFarAuto — Far Zone Autonomous (Blue Alliance)
//
// Mirrored from RedFarAuto via FieldMirror (144 - x, same y, π - heading)
//
// FLYWHEEL: runs at VELOCITY_FAR throughout — no idle between cycles
//
// SHOOT POSE HEADINGS (mirrored from red):
//   Shot 1 (Preload)  → SHOOT_POSE          ~113°
//   Shot 2 (Corner1)  → SHOOT_POSE_RETURN   ~121°
//   Shot 3 (Spike)    → SHOOT_POSE_RETURN_2 ~117°
//   Shot 4 (Corner2)  → SHOOT_POSE_RETURN_3 ~121°
//
// CHASSIS SPEED CHANGES:
//   SpikeMarkCycle:
//     - Approach curve       → NORMAL_SPEED
//     - Collect sweep        → INTAKE_SPEED  (slow)
//     - Return to shoot      → SPIKE_RETURN_SPEED (0.75)
//   CornerCycle (x2):
//     - Approach             → NORMAL_SPEED
//     - Bump forward/back    → INTAKE_SPEED  (slow)
//     - Return               → NORMAL_SPEED
// ═══════════════════════════════════════════════════════════════════════════

@Autonomous(name = "-BLUE FAR AUTO- SPIKE COLLECT")
public class BlueFarAuto extends AutoBase {

    // ─────────────────────────────────────────────────────────────────────
    // TIMING CONSTANTS
    // ─────────────────────────────────────────────────────────────────────
    private static final int SPINUP_MS         = 1600;
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
    private static final Pose START_POSE = FieldMirror.mirror(new Pose(80, 9, Math.toRadians(90)));

    // ─────────────────────────────────────────────────────────────────────
    // SHOOT POSES — mirrored from red (SHOOT_POSE_RETURN x corrected to 85)
    // ─────────────────────────────────────────────────────────────────────
    static final Pose SHOOT_POSE          = FieldMirror.mirror(new Pose(85.000, 14.000, Math.toRadians(67))); // Shot 1 — preload
    static final Pose SHOOT_POSE_RETURN   = FieldMirror.mirror(new Pose(88.000, 14.000, Math.toRadians(59))); // Shot 2 — Corner1 (x fixed from 8.0)
    static final Pose SHOOT_POSE_RETURN_2 = FieldMirror.mirror(new Pose(86.000, 14.000, Math.toRadians(63))); // Shot 3 — Spike
    static final Pose SHOOT_POSE_RETURN_3 = FieldMirror.mirror(new Pose(88.000, 14.000, Math.toRadians(59))); // Shot 4 — Corner2

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

                        // ── STEP 1: Spin up flywheel + set hood
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

        static final Pose CORNER_APPROACH_POSE  = FieldMirror.mirror(new Pose(130.000,  8.700, Math.toRadians(0)));
        static final Pose CORNER_BUMP_BACK_POSE = FieldMirror.mirror(new Pose(126.000,  8.700, Math.toRadians(0)));
        static final Pose CORNER_BUMP_FWD_POSE  = FieldMirror.mirror(new Pose(130.000,  8.600, Math.toRadians(0)));
        static final Pose CORNER_ADJUST_POSE    = FieldMirror.mirror(new Pose(122.000, 12.600, Math.toRadians(0)));
        static final Pose SPIKE_APPROACH_POSE   = FieldMirror.mirror(new Pose( 99.952, 36.000, Math.toRadians(0)));
        static final Pose SPIKE_SWEEP_END_POSE  = FieldMirror.mirror(new Pose(127.433, 36.000, Math.toRadians(0)));
        static final Pose PARK_POSE             = FieldMirror.mirror(new Pose(112.749,  8.708, Math.toRadians(0)));

        public PathChain PreloadShoot;
        public PathChain SpikeApproach;
        public PathChain SpikeSweep;
        public PathChain SpikeReturn;
        public PathChain CornerApproach;
        public PathChain CornerApproach2;
        public PathChain CornerBumpForward;
        public PathChain CornerBumpBack;
        public PathChain CornerAdjust;
        public PathChain CornerReturn1;
        public PathChain CornerReturn2;
        public PathChain Park;

        public Paths(Follower follower) {

            PreloadShoot = follower.pathBuilder()
                    .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            // ── Corner Cycle 1 — departs from SHOOT_POSE_RETURN ──
            CornerApproach = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE_RETURN, CORNER_APPROACH_POSE))
                    .setLinearHeadingInterpolation(SHOOT_POSE_RETURN.getHeading(), CORNER_APPROACH_POSE.getHeading())
                    .build();

            CornerBumpBack = follower.pathBuilder()
                    .addPath(new BezierLine(CORNER_APPROACH_POSE, CORNER_BUMP_BACK_POSE))
                    .setConstantHeadingInterpolation(Math.PI)
                    .build();

            CornerBumpForward = follower.pathBuilder()
                    .addPath(new BezierLine(CORNER_BUMP_BACK_POSE, CORNER_BUMP_FWD_POSE))
                    .setConstantHeadingInterpolation(Math.PI)
                    .build();

            CornerAdjust = follower.pathBuilder()
                    .addPath(new BezierLine(CORNER_BUMP_FWD_POSE, CORNER_ADJUST_POSE))
                    .setConstantHeadingInterpolation(Math.PI)
                    .build();

            CornerReturn1 = follower.pathBuilder()
                    .addPath(new BezierLine(CORNER_ADJUST_POSE, SHOOT_POSE_RETURN))
                    .setLinearHeadingInterpolation(CORNER_ADJUST_POSE.getHeading(), SHOOT_POSE_RETURN.getHeading())
                    .build();

            // ── Spike Mark Cycle — departs from SHOOT_POSE_RETURN ──
            SpikeApproach = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            SHOOT_POSE_RETURN,
                            FieldMirror.mirror(new Pose(90.434, 36.425)),
                            SPIKE_APPROACH_POSE))
                    .setLinearHeadingInterpolation(SHOOT_POSE_RETURN.getHeading(), SPIKE_APPROACH_POSE.getHeading())
                    .build();

            SpikeSweep = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE_APPROACH_POSE, SPIKE_SWEEP_END_POSE))
                    .setLinearHeadingInterpolation(SPIKE_APPROACH_POSE.getHeading(), SPIKE_SWEEP_END_POSE.getHeading())
                    .build();

            SpikeReturn = follower.pathBuilder()
                    .addPath(new BezierLine(SPIKE_SWEEP_END_POSE, SHOOT_POSE_RETURN_2))
                    .setLinearHeadingInterpolation(SPIKE_SWEEP_END_POSE.getHeading(), SHOOT_POSE_RETURN_2.getHeading())
                    .build();

            // ── Corner Cycle 2 — departs from SHOOT_POSE_RETURN_2 ──
            CornerApproach2 = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE_RETURN_2, CORNER_APPROACH_POSE))
                    .setLinearHeadingInterpolation(SHOOT_POSE_RETURN_2.getHeading(), CORNER_APPROACH_POSE.getHeading())
                    .build();

            CornerReturn2 = follower.pathBuilder()
                    .addPath(new BezierLine(CORNER_ADJUST_POSE, SHOOT_POSE_RETURN_3))
                    .setLinearHeadingInterpolation(CORNER_ADJUST_POSE.getHeading(), SHOOT_POSE_RETURN_3.getHeading())
                    .build();

            Park = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE_RETURN_3, PARK_POSE))
                    .setLinearHeadingInterpolation(SHOOT_POSE_RETURN_3.getHeading(), PARK_POSE.getHeading())
                    .build();
        }
    }
}

