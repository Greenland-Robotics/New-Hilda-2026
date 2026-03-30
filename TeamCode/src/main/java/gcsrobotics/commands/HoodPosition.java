package gcsrobotics.commands;

public enum HoodPosition {

    CLOSE(0.0),     // TODO: tune servo position for close shot
    MEDIUM(0.0),    // TODO: tune servo position for medium shot
    TOP(0.0),       // TODO: tune servo position for top shot
    FAR(0.0);       // TODO: tune servo position for far shot

    public final double servoPosition;

    HoodPosition(double servoPosition) {
        this.servoPosition = servoPosition;
    }
}