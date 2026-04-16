package gcsrobotics.pedroPathing;


import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.geometry.Pose;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {


    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.8)
            .forwardZeroPowerAcceleration(-47.0)                                // TUNED: measured value
            .lateralZeroPowerAcceleration(-75.330)
            .headingPIDFCoefficients(new PIDFCoefficients(3.0, 0, 0.25, 0))    // D bumped to dampen heading overshoot
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.01, 0.015)) // restored — D was larger than P causing wiggle
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.014, 0, 0.0035, 0.6, 0))
            .centripetalScaling(0.0009);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(73.5)    // TUNED
            .yVelocity(53.0)    // TUNED
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);


    // ─────────────────────────────────────────────────────────────────────
    // PATH CONSTRAINTS
    // Parameters in order:
    //   tValueConstraint        — path completion threshold (0.995 = 99.5% done)
    //   velocityConstraint      — velocity at path end (in/s)
    //   translationalConstraint — translational end tolerance (inches)
    //   headingConstraint       — heading end tolerance (radians) — 1.5° = 0.0262 rad
    //   timeoutConstraint       — max path time (seconds)
    //   brakingStrength         — braking aggressiveness
    //   BEZIER_CURVE_SEARCH_LIMIT — leave at 10
    //   brakingStart            — when braking begins
    // ─────────────────────────────────────────────────────────────────────
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,               // tValueConstraint
            0.1,                 // velocityConstraint
            0.1,                 // translationalConstraint
            Math.toRadians(1.5), // headingConstraint — 1.5 degrees
            50,                  // timeoutConstraint
            2,                   // brakingStrength
            10,                  // BEZIER_CURVE_SEARCH_LIMIT
            1.5                  // brakingStart
    );


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.02)
            .strafePodX(-1.36)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .build();
    }


    // =====================================================
    // ROBOT SUBSYSTEM CONSTANTS
    // =====================================================


    // ---- Kickstand (CRServo — runs while button held) ----
    public static class Kickstand {
        public static final double DEPLOY_POWER  =  1.0;
        public static final double RETRACT_POWER = -1.0;
        public static final String MOTOR_NAME    = "kickstand";
    }


    // ---- Gate (Standard Servo) ----
    public static class Gate {
        public static final double OPEN_POSITION  = 0.6;
        public static final double CLOSE_POSITION = 0.0;
    }


    // ---- Hood (Axon MAX servo — alliance-independent) ----
    // A = CLOSE, X = MEDIUM, Y = TOP, B = FAR
    public static class Hood {
        public static final double CLOSE  = 1.0;
        public static final double MEDIUM = 0.67;
        public static final double TOP    = 0.63;
        public static final double FAR    = 0.6;
    }


    // ---- Flywheel (dual goBILDA 6000 RPM, shared shaft) ----
    // All velocities in RPM — converted to ticks/sec in OpModeBase
    public static class Flywheel {
        public static final double PIDF_F = 15.5;
        public static final double PIDF_P = 3.82;
        public static final double PIDF_I = 0.0;
        public static final double PIDF_D = 0.0;

        public static final int STABILIZE_MS      = 300;

        public static final double VELOCITY_IDLE    = 500;
        public static final double VELOCITY_CLOSE   = 2380;
        public static final double VELOCITY_MEDIUM  = 2500;
        public static final double VELOCITY_TOP     = 2680;
        public static final double VELOCITY_FAR     = 3250;
        public static final double VELOCITY_FAR_TOP = 3280; // far auto — slightly higher than teleop FAR

        public static final double FIRE_WINDOW_LOW  = -150.0;
        public static final double FIRE_WINDOW_HIGH =  300.0;

        public static final double BANG_BANG_THRESHOLD = 0.93;
        public static final double BANG_BANG_POWER     = 1.0;

        public static final double RPM_THRESHOLD       = 0.95;
        public static final long   FLYWHEEL_TIMEOUT_MS = 3000;
        public static final long   SHOOT_DURATION_MS   = 2000;
    }


    // ---- Snap-to shooting poses ----
    // Mirror formula: X_blue = 144 - X_red, Y_blue = Y_red, Heading_blue = -Heading_red
    public static class SnapPositions {

        // ---- RED alliance shooting poses ----
        public static final Pose RED_CLOSE      = new Pose(108,   115, Math.toRadians(48));
        public static final Pose RED_MEDIUM     = new Pose(90,     96, Math.toRadians(46));
        public static final Pose RED_TOP        = new Pose(73,     76, Math.toRadians(48.5));
        public static final Pose RED_FAR        = new Pose(77.4,  19.2, Math.toRadians(72.2));
        public static final Pose RED_FAR_START  = new Pose(113,    9,  Math.toRadians(0));
        public static final Pose RED_NEAR_START = new Pose(124,  50,  Math.toRadians(45));

        // ---- RED utility poses ----
        public static final Pose RED_HUMAN_PLAYER = new Pose(36,  12, Math.toRadians(180));
        public static final Pose RED_PARK         = new Pose(37,  32, Math.toRadians(180));

        // ---- RED pose reset — robot backed against near wall ----
        public static final Pose RED_RESET_POSE   = new Pose(9.5,  8, Math.toRadians(180));

        // ---- BLUE alliance shooting poses ----
        public static final Pose BLUE_CLOSE      = new Pose(36,   115, Math.toRadians(132));
        public static final Pose BLUE_MEDIUM     = new Pose(54,    96, Math.toRadians(134));
        public static final Pose BLUE_TOP        = new Pose(71,    75, Math.toRadians(132));
        public static final Pose BLUE_FAR        = new Pose(77,   19.2, Math.toRadians(108));

        // ---- BLUE utility poses ----
        public static final Pose BLUE_HUMAN_PLAYER = new Pose(108, 12, Math.toRadians(0));
        public static final Pose BLUE_PARK         = new Pose(107, 32, Math.toRadians(0));
        public static final Pose BLUE_FAR_START    = new Pose(64,   9, Math.toRadians(90));
        public static final Pose BLUE_NEAR_START   = new Pose(21.5, 50, Math.toRadians(135));

        // ---- BLUE pose reset — robot backed against near wall ----
        public static final Pose BLUE_RESET_POSE   = new Pose(31,  9, Math.toRadians(180));
    }


    // ---- Intake ----
    public static class Intake {
        public static final double FORWARD_POWER = -1.0;
        public static final double REVERSE_POWER =  1.0;
    }


    // ---- LED (goBILDA RGB — configured as Servo) ----
    public static class LED {
        public static final double SOLID_RED    = 0.278; // verify on hardware — may need ±0.02 tuning
        public static final double SOLID_GREEN  = 0.500;
        public static final double SOLID_YELLOW = 0.388;
        public static final double OFF          = 0.0;
    }
}


