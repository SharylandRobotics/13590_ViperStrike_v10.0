package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class FieldLocalization {
    /**
     * Position of the farthest corner of the Blue OZ which still counts as a park
     */
    public final Position BlueOZ = new Position(DistanceUnit.INCH, -35.8, 53.4, 0, 0);
    /**
     * Position of the Blue Rung down the middle
     */
    public final Position BlueRung = new Position(DistanceUnit.INCH, -6.7, 29, 0, 0);
    /**
     * Position of the farthest corner of the Red OZ which still counts as a park
     */
    public final Position RedOZ = new Position(DistanceUnit.INCH, 36.2, -54.9,  0,0);
    /**
     * Position of the Red Rung down the middle
     */
    public final Position RedRung = new Position(DistanceUnit.INCH, 9.7, -27.7, 0, 0);

    /**
     * Position of the closest Blue Sample to the center of the field
     */
    public Position firstBlueSample;
    /**
     * Position of the closest Red Sample to the center of the field
     */
    public Position firstRedSample;
    /**
     * Position of the closes Blue Neutral Sample to the center of the field
     */
    public Position firstBlueNeutral;
    /**
     * Position of the closes Red Neutral Sample to the center of the field
     */
    public Position firstRedNeutral;

}
