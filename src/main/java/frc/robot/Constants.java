package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Constants {

    public static final Joystick cJoystick = new Joystick(0);

    public static final int cBaseArmID = 5;
    public static final int cMiddleArmID = 9;

    public static final int cArmPIDLoopID = 0;
    public static final int cTimeoutMs = 30;

    public static final double cArmP = 1;
    public static final double cArmI = 0;
    public static final double cArmD = 0;

}
