package gcsrobotics.commands;

public enum ShootingPosition {

    // =====================================================
    // BLUE SIDE
    // =====================================================
    // TODO: Replace all 0.0 values with real field coordinates
    // from the Localization Test in your Tuning OpMode

    CLOSE_BLUE(
            2400,   // target RPM
            0.0,    // TODO: hood servo position
            0.0,    // TODO: pose X (inches)
            0.0,    // TODO: pose Y (inches)
            0.0     // TODO: pose heading (radians)
    ),
    MEDIUM_BLUE(
            3000,   // target RPM
            0.0,    // TODO: hood servo position
            0.0,    // TODO: pose X
            0.0,    // TODO: pose Y
            0.0     // TODO: pose heading
    ),
    TOP_BLUE(
            3600,   // target RPM
            0.0,    // TODO: hood servo position
            0.0,    // TODO: pose X
            0.0,    // TODO: pose Y
            0.0     // TODO: pose heading
    ),
    FAR_BLUE(
            0,      // TODO: confirm FAR RPM
            0.0,    // TODO: hood servo position
            0.0,    // TODO: pose X
            0.0,    // TODO: pose Y
            0.0     // TODO: pose heading
    ),

    // =====================================================
    // RED SIDE
    // =====================================================
    // TODO: Mirror from blue once blue coords are confirmed
    // Formula:
    //   redX       = 144 - blueX
    //   redY       = blueY
    //   redHeading = Math.PI - blueHeading

    CLOSE_RED(
            2400,   // target RPM (same as blue)
            0.0,    // TODO: hood servo position (same as blue)
            0.0,    // TODO: 144 - CLOSE_BLUE.poseX
            0.0,    // TODO: CLOSE_BLUE.poseY
            0.0     // TODO: Math.PI - CLOSE_BLUE.poseHeading
    ),
    MEDIUM_RED(
            3000,
            0.0,    // TODO: hood servo position
            0.0,    // TODO: 144 - MEDIUM_BLUE.poseX
            0.0,    // TODO: MEDIUM_BLUE.poseY
            0.0     // TODO: Math.PI - MEDIUM_BLUE.poseHeading
    ),
    TOP_RED(
            3600,
            0.0,    // TODO: hood servo position
            0.0,    // TODO: 144 - TOP_BLUE.poseX
            0.0,    // TODO: TOP_BLUE.poseY
            0.0     // TODO: Math.PI - TOP_BLUE.poseHeading
    ),
    FAR_RED(
            0,      // TODO: confirm FAR RPM
            0.0,    // TODO: hood servo position
            0.0,    // TODO: 144 - FAR_BLUE.poseX
            0.0,    // TODO: FAR_BLUE.poseY
            0.0     // TODO: Math.PI - FAR_BLUE.poseHeading
    );

    // =====================================================
    // FIELDS
    // =====================================================
    public final double targetRPM;
    public final double hoodPosition;
    public final double poseX;
    public final double poseY;
    public final double poseHeading;

    ShootingPosition(
            double targetRPM,
            double hoodPosition,
            double poseX,
            double poseY,
            double poseHeading
    ) {
        this.targetRPM     = targetRPM;
        this.hoodPosition  = hoodPosition;
        this.poseX         = poseX;
        this.poseY         = poseY;
        this.poseHeading   = poseHeading;
    }

    // =====================================================
    // HELPERS
    // =====================================================

    /// Returns true if this is a blue alliance position
    public boolean isBlue() {
        return this.name().endsWith("_BLUE");
    }

    /// Returns true if this is a red alliance position
    public boolean isRed() {
        return this.name().endsWith("_RED");
    }

    /// Returns the mirrored position for the opposite alliance
    public ShootingPosition mirror() {
        switch (this) {
            case CLOSE_BLUE:  return CLOSE_RED;
            case MEDIUM_BLUE: return MEDIUM_RED;
            case TOP_BLUE:    return TOP_RED;
            case FAR_BLUE:    return FAR_RED;
            case CLOSE_RED:   return CLOSE_BLUE;
            case MEDIUM_RED:  return MEDIUM_BLUE;
            case TOP_RED:     return TOP_BLUE;
            case FAR_RED:     return FAR_BLUE;
            default:          return this;
        }
    }
}