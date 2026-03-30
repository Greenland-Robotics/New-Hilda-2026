package gcsrobotics.pedroPathing;

import com.pedropathing.geometry.Pose;

public class FieldMirror {

    private static final double FIELD_WIDTH = 144.0;

    public static Pose mirror(Pose blue) {
        return new Pose(
                FIELD_WIDTH - blue.getX(),
                blue.getY(),
                Math.PI - blue.getHeading()
        );
    }
}