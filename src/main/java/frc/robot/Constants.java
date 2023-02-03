package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Constants {

    public static final Joystick cJoystick = new Joystick(0);
    public static final double cJoystickMin = 0.1;

    public static final int cBaseArmID = 5;
    public static final int cMiddleArmID = 9;

    public static final int cArmPIDLoopID = 0;
    public static final int cTimeoutMs = 30;

    public static final double cArmP = 1;
    public static final double cArmI = 0;
    public static final double cArmD = 0;

    public static final double cDriveSpeed = 5;
    public static final double cTurnSpeed = 10;

    public static Trigger zeroGyro = new Trigger(() -> cJoystick.getRawButtonPressed(8));

}
