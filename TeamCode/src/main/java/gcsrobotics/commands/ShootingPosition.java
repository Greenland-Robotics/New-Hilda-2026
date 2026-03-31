package gcsrobotics.commands;

import gcsrobotics.pedroPathing.Constants;

public enum ShootingPosition {

    // =====================================================
    // Hood positions and flywheel velocities are
    // alliance-independent — same values for blue and red.
    // Velocities in rad/sec — use with AngleUnit.RADIANS.
    // TODO: Replace pose values after field measurement.
    // =====================================================

    CLOSE(
            Constants.Flywheel.VELOCITY_CLOSE,  // 2400 RPM → 251.3 rad/sec
            Constants.Hood.CLOSE,               // 0.10
            0.0,    // TODO: pose X (inches)
            0.0,    // TODO: pose Y (inches)
            0.0     // TODO: pose heading (radians)
    ),
    MEDIUM(
            Constants.Flywheel.VELOCITY_MEDIUM, // 3000 RPM → 314.2 rad/sec
            Constants.Hood.MEDIUM,              // 0.25
            0.0,    // TODO: pose X
            0.0,    // TODO: pose Y
            0.0     // TODO: pose heading
    ),
    TOP(
            Constants.Flywheel.VELOCITY_TOP,    // 3600 RPM → 377.0 rad/sec
            Constants.Hood.TOP,                 // 0.40
            0.0,    // TODO: pose X
            0.0,    // TODO: pose Y
            0.0     // TODO: pose heading
    ),
    FAR(
            Constants.Flywheel.VELOCITY_FAR,    // 4200 RPM → 439.8 rad/sec ⚠️ TBD
            Constants.Hood.FAR,                 // 0.55
            0.0,    // TODO: pose X
            0.0,    // TODO: pose Y
            0.0     // TODO: pose heading
    );

    // =====================================================
    // FIELDS
    // =====================================================
    public final double targetVelocity; // rad/sec
    public final double hoodPosition;   // Axon MAX servo position 0.0-1.0
    public final double poseX;          // TODO: field X inches
    public final double poseY;          // TODO: field Y inches
    public final double poseHeading;    // TODO: field heading radians

    ShootingPosition(
            double targetVelocity,
            double hoodPosition,
            double poseX,
            double poseY,
            double poseHeading
    ) {
        this.targetVelocity = targetVelocity;
        this.hoodPosition   = hoodPosition;
        this.poseX          = poseX;
        this.poseY          = poseY;
        this.poseHeading    = poseHeading;
    }
}