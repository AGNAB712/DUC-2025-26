package org.firstinspires.ftc.teamcode.auto.path_noGate;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Poses {
    //all in terms of blue side
    static public Pose startPosition = new Pose(29.280, 130.800, Math.toRadians(90));
    static public Pose startFarPosition = new Pose(56, 7, Math.toRadians(90));
    static public Pose shootPosition = new Pose(56.880+6, 84.000-6, Math.toRadians(135));
    static public Pose shootFarPosition = new Pose(56.880, 17.75, Math.toRadians(110));
}
