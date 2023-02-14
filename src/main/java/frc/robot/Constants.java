package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.fabrik.Point;

public class Constants {

    public static final Joystick cJoystick = new Joystick(0);
    public static final XboxController cXbox = new XboxController(1);
    public static final double cJoystickMin = 0.1;
    public static final double cXboxMin = 0.15;

    public static final int cBaseArmID = 9;
    public static final int cMiddleArmID = 10;

    public static final double cGoalXMin = 0;
    public static final double cGoalYMin = 0;
    public static final double cGoalXMax = 45;
    public static final double cGoalYMax = 75;

    public static final int cArmPIDLoopID = 0;
    public static final int cTimeoutMs = 30;

    //TODO find upper arm value
    public static final double[] cArmLengths = {41, 42.5};

    //TODO find important encoder values
    public static final int cMastEncoderID = 0;
    public static final int cArmEncoderID = 1;
    public static final double cMastEncoderOffset = 0.59111;
    public static final double cArmEncoderOffset = 0.18389;
    public static final double cMastEncoderMax = 0;
    public static final double cMastEncoderMin = 0;
    public static final double cArmEncoderMax = 0;
    public static final double cArmEncoderMin = 0;

    //TODO separate arm segment PID values
    public static final double cMastP = 20;
    public static final double cMastI = 0.5;
    public static final double cMastD = 0;
    public static final double cArmP = 0.9;
    public static final double cArmI = 0.1;
    public static final double cArmD = 0.5;

    public static final double cDriveSpeed = 8;
    public static final double cTurnSpeed = 1.5;

    public static Trigger zeroGyro = new Trigger(() -> cJoystick.getRawButtonPressed(8)); //cJoystick.getRawButtonPressed(8));

    public static Trigger autoBalance = new Trigger(() -> cJoystick.getRawButton(11));
    public static Trigger align = new Trigger(() -> cJoystick.getRawButton(12));
    public static final int cArmEncoderClicks = 1;

    public static final double cTopConeX = 39.75;
    public static final double cTopConeY = 46;

    public static final double cTopCubeX = 39.75;
    public static final double cTopCubeY = 35.5;

    public static final double cBottomConeX = 22.75;
    public static final double cBottomConeY = 34;
    public static final int cArmClicksPerRotation = 1;


    public static final Point cTopCone = new Point(39.75, 46);
    public static final Point cTopCube = new Point(39.75, 35.5);
    public static final Point cBottomCone = new Point(22.75, 34);
    public static final Point cBottomCube = new Point(22.75, 23.5);
    public static final Point cArmOrigin = new Point(-17, 10);
    public static final double cArmBuffer = 7;

    public static final int cCompresser = 11;
    public static final int cSolenoidFWD = 1;
    public static final int cSolenoidREV = 0;

}
