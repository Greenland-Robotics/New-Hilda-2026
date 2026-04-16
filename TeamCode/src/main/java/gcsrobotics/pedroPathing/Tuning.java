package gcsrobotics.pedroPathing;


import static gcsrobotics.pedroPathing.Tuning.changes;
import static gcsrobotics.pedroPathing.Tuning.drawCurrent;
import static gcsrobotics.pedroPathing.Tuning.drawCurrentAndHistory;
import static gcsrobotics.pedroPathing.Tuning.follower;
import static gcsrobotics.pedroPathing.Tuning.stopRobot;
import static gcsrobotics.pedroPathing.Tuning.telemetryM;


import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;


import android.annotation.SuppressLint;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.ArrayList;
import java.util.List;


/**
 * This is the Tuning class. It contains a selection menu for various tuning OpModes.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/26/2025
 */
@Configurable
@TeleOp(name = "Tuning", group = "Pedro Pathing")
public class Tuning extends SelectableOpMode {
    public static Follower follower;


    @IgnoreConfigurable
    static PoseHistory poseHistory;


    @IgnoreConfigurable
    static TelemetryManager telemetryM;


    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();


    public Tuning() {
        super("Select a Tuning OpMode", s -> {
            s.folder("Localization", l -> {
                l.add("Localization Test", LocalizationTest::new);
                l.add("Offsets Tuner", OffsetsTuner::new);
                l.add("Forward Tuner", ForwardTuner::new);
                l.add("Lateral Tuner", LateralTuner::new);
                l.add("Turn Tuner", TurnTuner::new);
            });
            s.folder("Automatic", a -> {
                a.add("Forward Velocity Tuner", ForwardVelocityTuner::new);
                a.add("Lateral Velocity Tuner", LateralVelocityTuner::new);
                a.add("Forward Zero Power Acceleration Tuner", ForwardZeroPowerAccelerationTuner::new);
                a.add("Lateral Zero Power Acceleration Tuner", LateralZeroPowerAccelerationTuner::new);
                a.add("Predictive Braking Tuner", PredictiveBrakingTuner::new);  // ← added
            });
            s.folder("Manual", p -> {
                p.add("Translational Tuner", TranslationalTuner::new);
                p.add("Heading Tuner", HeadingTuner::new);
                p.add("Drive Tuner", DriveTuner::new);
                p.add("Centripetal Tuner", CentripetalTuner::new);
            });
            s.folder("Tests", p -> {
                p.add("Line", Line::new);
                p.add("Triangle", Triangle::new);
                p.add("Circle", Circle::new);
            });
            s.folder("Swerve", p -> {
                p.add("Analog Min / Max Tuner", AnalogMinMaxTuner::new);
                p.add("Swerve Offsets Test", SwerveOffsetsTest::new);
                p.add("Swerve Turn Test", SwerveTurnTest::new);
            });
        });
    }


    @Override
    public void onSelect() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }


        follower.setStartingPose(new Pose());


        poseHistory = follower.getPoseHistory();


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    @Override
    public void onLog(List<String> lines) {}


    public static void drawCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }


    public static void drawCurrentAndHistory() {
        Drawing.drawPoseHistory(poseHistory);
        drawCurrent();
    }


    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0, 0, 0, true);
    }
}


// =====================================================
// LOCALIZATION TUNERS
// =====================================================


class LocalizationTest extends OpMode {
    boolean debugStringEnabled = false;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }


    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }
        telemetryM.debug("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic drive on gamepad 1.");
        telemetryM.debug("Drivetrain debug string " + (((debugStringEnabled) ? "enabled" : "disabled")) +
                " (press gamepad a to toggle)");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }


    @Override
    public void loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        if (debugStringEnabled) {
            telemetryM.debug("Drivetrain Debug String:\n" + follower.getDrivetrain().debugString());
        }
        telemetryM.update(telemetry);
        drawCurrentAndHistory();
    }
}


/**
 * OffsetsTuner — added from v2
 * Tracks robot movement as it turns 180 degrees and calculates
 * the correct strafeX and forwardY pod offsets.
 * Prerequisite: set both offsets to 0 in Constants before running.
 */
class OffsetsTuner extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
        follower.update();
        drawCurrent();
    }


    @Override
    public void init_loop() {
        telemetryM.debug("Prerequisite: Make sure both your offsets are set to 0 in your localizer constants.");
        telemetryM.debug("Turn your robot " + Math.PI + " radians. Your offsets in inches will be shown on the telemetry.");
        telemetryM.update(telemetry);
        drawCurrent();
    }


    @Override
    public void loop() {
        follower.update();
        telemetryM.debug("Total Angle: " + follower.getTotalHeading());
        telemetryM.debug("The following values are the offsets in inches that should be applied to your localizer.");
        telemetryM.debug("strafeX: " + ((72.0 - follower.getPose().getX()) / 2.0));
        telemetryM.debug("forwardY: " + ((72.0 - follower.getPose().getY()) / 2.0));
        telemetryM.update(telemetry);
        drawCurrentAndHistory();
    }
}


class ForwardTuner extends OpMode {
    public static double DISTANCE = 48;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
        follower.update();
        drawCurrent();
    }


    @Override
    public void init_loop() {
        telemetryM.debug("Pull your robot forward " + DISTANCE + " inches. Your forward ticks to inches will be shown on the telemetry.");
        telemetryM.update(telemetry);
        drawCurrent();
    }


    @Override
    public void loop() {
        follower.update();
        telemetryM.debug("Distance Moved: " + follower.getPose().getX());
        telemetryM.debug("The multiplier will display what your forward ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
        telemetryM.debug("Multiplier: " + (DISTANCE / (follower.getPose().getX() / follower.getPoseTracker().getLocalizer().getForwardMultiplier())));
        telemetryM.update(telemetry);
        drawCurrentAndHistory();
    }
}


class LateralTuner extends OpMode {
    public static double DISTANCE = 48;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
        follower.update();
        drawCurrent();
    }


    @Override
    public void init_loop() {
        telemetryM.debug("Pull your robot to the right " + DISTANCE + " inches. Your strafe ticks to inches will be shown on the telemetry.");
        telemetryM.update(telemetry);
        drawCurrent();
    }


    @Override
    public void loop() {
        follower.update();
        telemetryM.debug("Distance Moved: " + follower.getPose().getY());
        telemetryM.debug("The multiplier will display what your strafe ticks to inches should be to scale your current distance to " + DISTANCE + " inches.");
        telemetryM.debug("Multiplier: " + (DISTANCE / (follower.getPose().getY() / follower.getPoseTracker().getLocalizer().getLateralMultiplier())));
        telemetryM.update(telemetry);
        drawCurrentAndHistory();
    }
}


class TurnTuner extends OpMode {
    public static double ANGLE = 2 * Math.PI;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
        follower.update();
        drawCurrent();
    }


    @Override
    public void init_loop() {
        telemetryM.debug("Turn your robot " + ANGLE + " radians. Your turn ticks to inches will be shown on the telemetry.");
        telemetryM.update(telemetry);
        drawCurrent();
    }


    @Override
    public void loop() {
        follower.update();
        telemetryM.debug("Total Angle: " + follower.getTotalHeading());
        telemetryM.debug("The multiplier will display what your turn ticks to inches should be to scale your current angle to " + ANGLE + " radians.");
        telemetryM.debug("Multiplier: " + (ANGLE / (follower.getTotalHeading() / follower.getPoseTracker().getLocalizer().getTurningMultiplier())));
        telemetryM.update(telemetry);
        drawCurrentAndHistory();
    }
}


// =====================================================
// AUTOMATIC TUNERS
// =====================================================


class ForwardVelocityTuner extends OpMode {
    private final ArrayList<Double> velocities = new ArrayList<>();
    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;
    private boolean end;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }


    @Override
    public void init_loop() {
        telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.");
        telemetryM.debug("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetryM.debug("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.");
        telemetryM.debug("Press B on game pad 1 to stop.");
        telemetryM.debug("pose", follower.getPose());
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }
        follower.startTeleopDrive(true);
        follower.update();
        end = false;
    }


    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }
        follower.update();
        drawCurrentAndHistory();
        if (!end) {
            if (Math.abs(follower.getPose().getX()) > (DISTANCE + 72)) {
                end = true;
                stopRobot();
            } else {
                follower.setTeleOpDrive(1, 0, 0, true);
                double currentVelocity = Math.abs(follower.poseTracker.getLocalizer().getVelocity().getX());
                velocities.add(currentVelocity);
                velocities.remove(0);
            }
        } else {
            stopRobot();
            double average = 0;
            for (double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();
            telemetryM.debug("Forward Velocity: " + average);
            telemetryM.debug("\n");
            telemetryM.debug("Press A to set the Forward Velocity temporarily (while robot remains on).");
            for (int i = 0; i < velocities.size(); i++) {
                telemetry.addData(String.valueOf(i), velocities.get(i));
            }
            telemetryM.update(telemetry);
            telemetry.update();
            if (gamepad1.aWasPressed()) {
                follower.setXVelocity(average);
                String message = "XMovement: " + average;
                changes.add(message);
            }
        }
    }
}


class LateralVelocityTuner extends OpMode {
    private final ArrayList<Double> velocities = new ArrayList<>();
    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;
    private boolean end;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }


    @Override
    public void init_loop() {
        telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches to the right.");
        telemetryM.debug("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetryM.debug("After running the distance, the robot will cut power from the drivetrain and display the strafe velocity.");
        telemetryM.debug("Press B on Gamepad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }
        follower.startTeleopDrive(true);
        follower.update();
    }


    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }
        follower.update();
        drawCurrentAndHistory();
        if (!end) {
            if (Math.abs(follower.getPose().getY()) > (DISTANCE + 72)) {
                end = true;
                stopRobot();
            } else {
                follower.setTeleOpDrive(0, 1, 0, true);
                double currentVelocity = Math.abs(follower.getVelocity().dot(new Vector(1, Math.PI / 2)));
                velocities.add(currentVelocity);
                velocities.remove(0);
            }
        } else {
            stopRobot();
            double average = 0;
            for (double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();
            telemetryM.debug("Strafe Velocity: " + average);
            telemetryM.debug("\n");
            telemetryM.debug("Press A to set the Lateral Velocity temporarily (while robot remains on).");
            telemetryM.update(telemetry);
            if (gamepad1.aWasPressed()) {
                follower.setYVelocity(average);
                String message = "YMovement: " + average;
                changes.add(message);
            }
        }
    }
}


class ForwardZeroPowerAccelerationTuner extends OpMode {
    private final ArrayList<Double> accelerations = new ArrayList<>();
    public static double VELOCITY = 30;
    private double previousVelocity;
    private long previousTimeNano;
    private boolean stopping;
    private boolean end;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }


    @Override
    public void init_loop() {
        telemetryM.debug("The robot will run forward until it reaches " + VELOCITY + " inches per second.");
        telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.debug("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.");
        telemetryM.debug("Press B on Gamepad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        follower.startTeleopDrive(false);
        follower.update();
        follower.setTeleOpDrive(1, 0, 0, true);
    }


    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }
        follower.update();
        drawCurrentAndHistory();
        Vector heading = new Vector(1.0, follower.getPose().getHeading());
        if (!end) {
            if (!stopping) {
                if (follower.getVelocity().dot(heading) > VELOCITY) {
                    previousVelocity = follower.getVelocity().dot(heading);
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    follower.setTeleOpDrive(0, 0, 0, true);
                }
            } else {
                double currentVelocity = follower.getVelocity().dot(heading);
                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();
                if (currentVelocity < follower.getConstraints().getVelocityConstraint()) {
                    end = true;
                }
            }
        } else {
            double average = 0;
            for (double acceleration : accelerations) {
                average += acceleration;
            }
            average /= accelerations.size();
            telemetryM.debug("Forward Zero Power Acceleration (Deceleration): " + average);
            telemetryM.debug("\n");
            telemetryM.debug("Press A to set the Forward Zero Power Acceleration temporarily (while robot remains on).");
            telemetryM.update(telemetry);
            if (gamepad1.aWasPressed()) {
                follower.getConstants().setForwardZeroPowerAcceleration(average);
                String message = "Forward Zero Power Acceleration: " + average;
                changes.add(message);
            }
        }
    }
}


class LateralZeroPowerAccelerationTuner extends OpMode {
    private final ArrayList<Double> accelerations = new ArrayList<>();
    public static double VELOCITY = 30;
    private double previousVelocity;
    private long previousTimeNano;
    private boolean stopping;
    private boolean end;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }


    @Override
    public void init_loop() {
        telemetryM.debug("The robot will run to the right until it reaches " + VELOCITY + " inches per second.");
        telemetryM.debug("Then, it will cut power from the drivetrain and roll to a stop.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.debug("After stopping, the lateral zero power acceleration (natural deceleration) will be displayed.");
        telemetryM.debug("Press B on game pad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        follower.startTeleopDrive(false);
        follower.update();
        follower.setTeleOpDrive(0, 1, 0, true);
    }


    @Override
    public void loop() {
        if (gamepad1.bWasPressed()) {
            stopRobot();
            requestOpModeStop();
        }
        follower.update();
        drawCurrentAndHistory();
        Vector heading = new Vector(1.0, follower.getPose().getHeading() - Math.PI / 2);
        if (!end) {
            if (!stopping) {
                if (Math.abs(follower.getVelocity().dot(heading)) > VELOCITY) {
                    previousVelocity = Math.abs(follower.getVelocity().dot(heading));
                    previousTimeNano = System.nanoTime();
                    stopping = true;
                    follower.setTeleOpDrive(0, 0, 0, true);
                }
            } else {
                double currentVelocity = Math.abs(follower.getVelocity().dot(heading));
                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();
                if (currentVelocity < follower.getConstraints().getVelocityConstraint()) {
                    end = true;
                }
            }
        } else {
            double average = 0;
            for (double acceleration : accelerations) {
                average += acceleration;
            }
            average /= accelerations.size();
            telemetryM.debug("Lateral Zero Power Acceleration (Deceleration): " + average);
            telemetryM.debug("\n");
            telemetryM.debug("Press A to set the Lateral Zero Power Acceleration temporarily (while robot remains on).");
            telemetryM.update(telemetry);
            if (gamepad1.aWasPressed()) {
                follower.getConstants().setLateralZeroPowerAcceleration(average);
                String message = "Lateral Zero Power Acceleration: " + average;
                changes.add(message);
            }
        }
    }
}


/**
 * PredictiveBrakingTuner — added from v2
 * Runs robot forward/backward at various powers, measures stopping distances,
 * fits a quadratic braking model, and reports kFriction and kBraking for
 * manual entry into Constants.java.
 */
class PredictiveBrakingTuner extends OpMode {
    private static final double[] TEST_POWERS =
            {1, 1, 1, 0.9, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2};
    private static final double BRAKING_POWER = -0.2;
    private static final int DRIVE_TIME_MS = 1000;


    private enum State {
        START_MOVE, WAIT_DRIVE_TIME, APPLY_BRAKE, WAIT_BRAKE_TIME, RECORD, DONE
    }


    private static class BrakeRecord {
        double timeMs;
        Pose pose;
        double velocity;


        BrakeRecord(double timeMs, Pose pose, double velocity) {
            this.timeMs = timeMs;
            this.pose = pose;
            this.velocity = velocity;
        }
    }


    private State state = State.START_MOVE;
    private final ElapsedTime timer = new ElapsedTime();
    private int iteration = 0;
    private Vector startPosition;
    private double measuredVelocity;
    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private final List<BrakeRecord> brakeData = new ArrayList<>();


    @Override
    public void init() {}


    @Override
    public void init_loop() {
        telemetryM.debug("The robot will move forwards and backwards starting at max speed and slowing down.");
        telemetryM.debug("Make sure you have enough room. Leave at least 4-5 feet.");
        telemetryM.debug("After stopping, kFriction and kBraking will be displayed.");
        telemetryM.debug("Make sure to turn the timer off.");
        telemetryM.debug("Press B on game pad 1 to stop.");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        timer.reset();
        follower.update();
        follower.startTeleopDrive(true);
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        follower.update();
        if (gamepad1.b) {
            stopRobot();
            requestOpModeStop();
            return;
        }
        double direction = (iteration % 2 == 0) ? 1 : -1;
        switch (state) {
            case START_MOVE: {
                if (iteration >= TEST_POWERS.length) {
                    state = State.DONE;
                    break;
                }
                double currentPower = TEST_POWERS[iteration];
                follower.setMaxPower(currentPower);
                follower.setTeleOpDrive(direction, 0, 0, true);
                timer.reset();
                state = State.WAIT_DRIVE_TIME;
                break;
            }
            case WAIT_DRIVE_TIME: {
                if (timer.milliseconds() >= DRIVE_TIME_MS) {
                    measuredVelocity = follower.getVelocity().getMagnitude();
                    startPosition = follower.getPose().getAsVector();
                    state = State.APPLY_BRAKE;
                }
                break;
            }
            case APPLY_BRAKE: {
                follower.setTeleOpDrive(BRAKING_POWER * direction, 0, 0, true);
                timer.reset();
                state = State.WAIT_BRAKE_TIME;
                break;
            }
            case WAIT_BRAKE_TIME: {
                double t = timer.milliseconds();
                Pose currentPose = follower.getPose();
                double currentVelocity = follower.getVelocity().getMagnitude();
                brakeData.add(new BrakeRecord(t, currentPose, currentVelocity));
                if (follower.getVelocity().dot(new Vector(direction, follower.getHeading())) <= 0) {
                    state = State.RECORD;
                }
                break;
            }
            case RECORD: {
                Vector endPosition = follower.getPose().getAsVector();
                double brakingDistance = endPosition.minus(startPosition).getMagnitude();
                velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});
                telemetryM.debug("Test " + iteration,
                        String.format("v=%.3f  d=%.3f", measuredVelocity, brakingDistance));
                telemetryM.update(telemetry);
                iteration++;
                state = State.START_MOVE;
                break;
            }
            case DONE: {
                stopRobot();

                // Quadratic least-squares fit: d = kFriction*v + kBraking*v^2
                // (no constant term — braking distance is 0 at v=0)
                // Normal equations:
                // [sumV2  sumV3] [kFriction]   [sumDV ]
                // [sumV3  sumV4] [kBraking ] = [sumDV2]
                double sumV2 = 0, sumV3 = 0, sumV4 = 0, sumDV = 0, sumDV2 = 0;
                int n = velocityToBrakingDistance.size();

                for (double[] pair : velocityToBrakingDistance) {
                    double v = pair[0];
                    double d = pair[1];
                    sumV2  += v * v;
                    sumV3  += v * v * v;
                    sumV4  += v * v * v * v;
                    sumDV  += d * v;
                    sumDV2 += d * v * v;
                }

                double det = sumV2 * sumV4 - sumV3 * sumV3;
                double kFriction = (det != 0) ? (sumDV * sumV4 - sumDV2 * sumV3) / det : 0;
                double kBraking  = (det != 0) ? (sumV2 * sumDV2 - sumV3 * sumDV)  / det : 0;

                telemetryM.debug("=== Predictive Braking Results ===");
                telemetryM.debug(String.format("kFriction : %.6f", kFriction));
                telemetryM.debug(String.format("kBraking  : %.6f", kBraking));
                telemetryM.debug("");
                telemetryM.debug("Enter these into Constants.java under followerConstants:");
                telemetryM.debug(String.format("  .kFriction(%.6f)", kFriction));
                telemetryM.debug(String.format("  .kBraking(%.6f)",  kBraking));
                telemetryM.debug("");
                telemetryM.debug("--- Raw Data Points (" + n + " runs) ---");
                for (int i = 0; i < n; i++) {
                    telemetryM.debug(String.format(
                            "  [%2d] v=%.3f in/s  d=%.3f in",
                            i, velocityToBrakingDistance.get(i)[0], velocityToBrakingDistance.get(i)[1]));
                }
                telemetryM.update(telemetry);
                break;
            }
        }
    }
}


// =====================================================
// MANUAL TUNERS
// =====================================================


class TranslationalTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;
    private Path forwards;
    private Path backwards;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }


    @Override
    public void init_loop() {
        telemetryM.debug("This will activate the translational PIDF(s)");
        telemetryM.debug("The robot will try to stay in place while you push it laterally.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's translational PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        follower.deactivateAllPIDFs();
        follower.activateTranslational();
        forwards = new Path(new BezierLine(new Pose(72, 72), new Pose(DISTANCE + 72, 72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72, 72), new Pose(72, 72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }


    @Override
    public void loop() {
        follower.update();
        drawCurrentAndHistory();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }
        telemetryM.debug("Push the robot laterally to test the Translational PIDF(s).");
        telemetryM.addData("Zero Line", 0);
        telemetryM.addData("Error X", follower.errorCalculator.getTranslationalError().getXComponent());
        telemetryM.addData("Error Y", follower.errorCalculator.getTranslationalError().getYComponent());
        telemetryM.update(telemetry);
    }
}


class HeadingTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;
    private Path forwards;
    private Path backwards;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }


    @Override
    public void init_loop() {
        telemetryM.debug("This will activate the heading PIDF(s).");
        telemetryM.debug("The robot will try to stay at a constant heading while you try to turn it.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's heading PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        follower.deactivateAllPIDFs();
        follower.activateHeading();
        forwards = new Path(new BezierLine(new Pose(72, 72), new Pose(DISTANCE + 72, 72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72, 72), new Pose(72, 72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }


    @Override
    public void loop() {
        follower.update();
        drawCurrentAndHistory();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }
        telemetryM.debug("Turn the robot manually to test the Heading PIDF(s).");
        telemetryM.addData("Zero Line", 0);
        telemetryM.addData("Error", follower.errorCalculator.getHeadingError());
        telemetryM.update(telemetry);
    }
}


class DriveTuner extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;
    private PathChain forwards;
    private PathChain backwards;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }


    @Override
    public void init_loop() {
        telemetryM.debug("This will run the robot in a straight line going " + DISTANCE + " inches forward.");
        telemetryM.debug("The robot will go forward and backward continuously along the path.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        follower.deactivateAllPIDFs();
        follower.activateDrive();
        forwards = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(72, 72), new Pose(DISTANCE + 72, 72)))
                .setConstantHeadingInterpolation(0)
                .build();
        backwards = follower.pathBuilder()
                .setGlobalDeceleration()
                .addPath(new BezierLine(new Pose(DISTANCE + 72, 72), new Pose(72, 72)))
                .setConstantHeadingInterpolation(0)
                .build();
        follower.followPath(forwards);
    }


    @Override
    public void loop() {
        follower.update();
        drawCurrentAndHistory();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }
        telemetryM.debug("Driving forward?: " + forward);
        telemetryM.addData("Zero Line", 0);
        telemetryM.addData("Error", follower.errorCalculator.getDriveErrors()[1]);
        telemetryM.update(telemetry);
    }
}


class CentripetalTuner extends OpMode {
    public static double DISTANCE = 20;
    private boolean forward = true;
    private Path forwards;
    private Path backwards;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }


    @Override
    public void init_loop() {
        telemetryM.debug("This will run the robot in a curve going " + DISTANCE + " inches to the left and the same number of inches forward.");
        telemetryM.debug("The robot will go continuously along the path.");
        telemetryM.debug("Make sure you have enough room.");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        follower.activateAllPIDFs();
        forwards = new Path(new BezierCurve(
                new Pose(72, 72),
                new Pose(Math.abs(DISTANCE) + 72, 72),
                new Pose(Math.abs(DISTANCE) + 72, DISTANCE + 72)));
        backwards = new Path(new BezierCurve(
                new Pose(Math.abs(DISTANCE) + 72, DISTANCE + 72),
                new Pose(Math.abs(DISTANCE) + 72, 72),
                new Pose(72, 72)));
        backwards.setTangentHeadingInterpolation();
        backwards.reverseHeadingInterpolation();
        follower.followPath(forwards);
    }


    @Override
    public void loop() {
        follower.update();
        drawCurrentAndHistory();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }
        telemetryM.debug("Driving away from the origin along the curve?: " + forward);
        telemetryM.update(telemetry);
    }
}


// =====================================================
// TEST OPMODES
// =====================================================


class Line extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;
    private Path forwards;
    private Path backwards;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }


    @Override
    public void init_loop() {
        telemetryM.debug("This will activate all the PIDF(s)");
        telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        follower.activateAllPIDFs();
        forwards = new Path(new BezierLine(new Pose(72, 72), new Pose(DISTANCE + 72, 72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE + 72, 72), new Pose(72, 72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }


    @Override
    public void loop() {
        follower.update();
        drawCurrentAndHistory();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }
        telemetryM.debug("Driving Forward?: " + forward);
        telemetryM.update(telemetry);
    }
}


class Triangle extends OpMode {
    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    private final Pose interPose = new Pose(24 + 72, -24 + 72, Math.toRadians(90));
    private final Pose endPose   = new Pose(24 + 72,  24 + 72, Math.toRadians(45));
    private PathChain triangle;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }


    @Override
    public void init_loop() {
        telemetryM.debug("This will run in a roughly triangular shape, starting on the bottom-middle point.");
        telemetryM.debug("So, make sure you have enough space to the left, front, and right to run the OpMode.");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        follower.setStartingPose(startPose);
        triangle = follower.pathBuilder()
                .addPath(new BezierLine(startPose, interPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                .addPath(new BezierLine(interPose, endPose))
                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                .addPath(new BezierLine(endPose, startPose))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();
        follower.followPath(triangle);
    }


    @Override
    public void loop() {
        follower.update();
        drawCurrentAndHistory();
        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }
    }
}


class Circle extends OpMode {
    public static double RADIUS = 10;
    private PathChain circle;


    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }


    @Override
    public void init_loop() {
        telemetryM.debug("This will run in a roughly circular shape of radius " + RADIUS + ", starting on the right-most edge.");
        telemetryM.debug("So, make sure you have enough space to the left, front, and back to run the OpMode.");
        telemetryM.debug("It will also continuously face the center of the circle to test your heading and centripetal correction.");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        circle = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(72, 72), new Pose(RADIUS + 72, 72), new Pose(RADIUS + 72, RADIUS + 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .addPath(new BezierCurve(new Pose(RADIUS + 72, RADIUS + 72), new Pose(RADIUS + 72, (2 * RADIUS) + 72), new Pose(72, (2 * RADIUS) + 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .addPath(new BezierCurve(new Pose(72, (2 * RADIUS) + 72), new Pose(-RADIUS + 72, (2 * RADIUS) + 72), new Pose(-RADIUS + 72, RADIUS + 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .addPath(new BezierCurve(new Pose(-RADIUS + 72, RADIUS + 72), new Pose(-RADIUS + 72, 72), new Pose(72, 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(72, RADIUS + 72))
                .build();
        follower.followPath(circle);
    }


    @Override
    public void loop() {
        follower.update();
        drawCurrentAndHistory();
        if (follower.atParametricEnd()) {
            follower.followPath(circle);
        }
    }
}


// =====================================================
// SWERVE TUNERS
// =====================================================


class AnalogMinMaxTuner extends OpMode {
    public String[] encoderNames = {"leftFrontEncoder", "rightFrontEncoder", "leftBackEncoder", "rightBackEncoder"};
    public AnalogInput[] encoders = new AnalogInput[encoderNames.length];
    public double[] minVoltages = new double[encoderNames.length];
    public double[] maxVoltages = new double[encoderNames.length];
    public List<LynxModule> lynxModules;


    @Override
    public void init() {
        lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : lynxModules) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        for (int i = 0; i < encoders.length; i++) {
            encoders[i] = hardwareMap.get(AnalogInput.class, encoderNames[i]);
            minVoltages[i] = 5;
        }
    }


    @Override
    public void init_loop() {
        telemetryM.debug("Press START. Then, Spin each pod slowly for 4 to 5 full rotations.\n" +
                "The OpMode will keep track of the min and max voltages seen so far and print them to telemetry.");
        telemetryM.update(telemetry);
    }


    @Override
    public void start() {}


    @Override
    public void loop() {
        for (LynxModule hub : lynxModules) {
            hub.clearBulkCache();
        }
        telemetryM.debug("Spin each pod slowly for 4 to 5 full rotations.\n\n");
        for (int i = 0; i < encoders.length; i++) {
            double currentVoltage = encoders[i].getVoltage();
            minVoltages[i] = Math.min(minVoltages[i], currentVoltage);
            maxVoltages[i] = Math.max(maxVoltages[i], currentVoltage);
            telemetryM.addData(encoderNames[i] + " min value:", minVoltages[i]);
            telemetryM.addData(encoderNames[i] + " max value:", maxVoltages[i]);
            telemetryM.addLine("");
        }
        telemetryM.update();
    }
}


class SwerveOffsetsTest extends OpMode {
    boolean debugStringEnabled = false;


    @Override
    public void init() {}


    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }
        telemetryM.debug("This OpMode will run all four swerve pods in the direction they think is forward"
                + "\nensure your bot is not on the ground while running");
        telemetryM.debug("Drivetrain debug string " + (((debugStringEnabled) ? "enabled" : "disabled")) +
                " (press gamepad a to toggle)");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }


    @Override
    public void loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }
        follower.setTeleOpDrive(0.25, 0, 0, true);
        follower.update();
        if (debugStringEnabled) {
            telemetryM.debug("Drivetrain Debug String:\n" + follower.getDrivetrain().debugString());
        }
        telemetryM.update(telemetry);
        drawCurrentAndHistory();
    }
}


class SwerveTurnTest extends OpMode {
    boolean debugStringEnabled = false;


    @Override
    public void init() {}


    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }
        telemetryM.debug("This OpMode will run all four swerve pods in their turning direction (perpendicular to the center of the robot)"
                + "\nrun this once off the ground to check servo directions and motor directions before testing on the ground");
        telemetryM.debug("Drivetrain debug string " + (((debugStringEnabled) ? "enabled" : "disabled")) +
                " (press gamepad a to toggle)");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }


    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }


    @Override
    public void loop() {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            debugStringEnabled = !debugStringEnabled;
        }
        follower.setTeleOpDrive(0, 0, 0.25, true);
        follower.update();
        if (debugStringEnabled) {
            telemetryM.debug("Drivetrain Debug String:\n" + follower.getDrivetrain().debugString());
        }
        telemetryM.update(telemetry);
        drawCurrentAndHistory();
    }
}


// =====================================================
// DRAWING UTILITY
// =====================================================


class Drawing {
    public static final double ROBOT_RADIUS = 9;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private static final Style robotLook   = new Style("", "#3F51B5", 0.75);
    private static final Style historyLook = new Style("", "#4CAF50", 0.75);


    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }


    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(),
                    follower.getCurrentPath().getHeadingGoal(
                            follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);
        sendPacket();
    }


    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }
        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);
        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(),     y2 = pose.getY() + v.getYComponent();
        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }


    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }


    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();
        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }
        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }


    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }


    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);
        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {
            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }


    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }


    public static void sendPacket() {
        panelsField.update();
    }
}


