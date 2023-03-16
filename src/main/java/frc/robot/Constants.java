package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.fabrik.Point;

public class Constants {

    public static final Joystick cJoystick = new Joystick(0);
    public static final XboxController cXbox = new XboxController(1);

    public static final Joystick cButtonPanel = new Joystick(2);

    public static final Trigger[] cPanelButtons;

    static {
        cPanelButtons = new Trigger[12];

        for(int i = 0; i < cPanelButtons.length; i++) {
            int finalI = i+1;
            cPanelButtons[i] = new Trigger(() -> cButtonPanel.getRawButton(finalI));
        }
    }

    public static final double cJoystickMin = 0.1;
    public static final double cXboxMin = 0.15;

    public static final int cBaseArmID = 10;
    public static final int cMiddleArmID = 9;

    public static final double cGoalXMin = 0;
    public static final double cGoalYMin = 0;
    public static final double cGoalXMax = 45;
    public static final double cGoalYMax = 75;

    public static final int cArmPIDLoopID = 0;
    public static final int cTimeoutMs = 30;
    public static final double[] cArmLengths = {41, 46};
    public static final int cMastEncoderID = 0;
    public static final int cArmEncoderID = 1;
    public static final int cArmClicksPerRotation = 1;
    public static final double cMastEncoderOffset = 0.59111;
    public static final double cArmEncoderOffset = 0.519; //19
    public static final double cMastEncoderMax = (0.867 - cMastEncoderOffset) * ((Math.PI * 2) / cArmClicksPerRotation);
    public static final double cMastEncoderMin = (0.732 - cMastEncoderOffset) * ((Math.PI * 2) / cArmClicksPerRotation);

    //public static final double cArmEncoderMax = (0.5 - cArmEncoderOffset) * ((Math.PI * 2) / cArmClicksPerRotation);
    //public static final double cArmEncoderMin = (0.23 - cArmEncoderOffset) * ((Math.PI * 2) / cArmClicksPerRotation);

    public static final double cArmEncoderMax = Math.toRadians(117);
    public static final double cArmEncoderMin = Math.toRadians(15);//

    public static final double cMastP = 16;
    public static final double cMastI = 0;
    public static final double cMastD = 1;
    public static final double cPeakMastOutputForward = 0.1;
    public static final double cPeakMastOutputBackward = -0.4;
    public static final double cArmP = 1;
    public static final double cArmI = 0;
    public static final double cArmD = 0;
    public static final double cPeakArmOutputForward = 0.7;
    public static final double cPeakArmOutputBackward = -0.5;

    public static final double cDriveSpeed = 5;
    public static final double onlyDriverSpeed = 1.6;
    public static final double cTurnSpeed = 3;

    public static Trigger cXboxStart = new Trigger(() -> cXbox.getStartButtonPressed());

    public static Trigger cXboxRightBumper = new Trigger(() -> cXbox.getRightBumperPressed());

    public static Trigger zeroGyroJoystick = new Trigger(() -> cJoystick.getRawButtonPressed(8));

    public static Trigger autoBalance = new Trigger(() -> cXbox.getRawButton(7));
    public static Trigger cXboxA = new Trigger(() -> cXbox.getAButton());
    public static Trigger cXboxB = new Trigger(() -> cXbox.getBButton());
    public static Trigger cXboxX = new Trigger(() -> cXbox.getXButton());
    public static Trigger cXboxY = new Trigger(() -> cXbox.getYButton());
    public static final int cArmEncoderClicks = 1;

    public static final double cTopConeX = 39.75;
    public static final double cTopConeY = 46;

    public static final double cTopCubeX = 39.75;
    public static final double cTopCubeY = 35.5;

    public static final double cBottomConeX = 22.75;
    public static final double cBottomConeY = 34;

    public static final double cArmBufferX = 7;
    public static final double cArmBufferY = 7;

    public static final Point cTestPoint = new Point(cArmLengths[0], cArmLengths[1]);
    public static final Point cTopCone = new Point(41 /* + cArmBufferX*/, 56 /*+ cArmBufferY*/);
    public static final Point cTopCube = new Point(39.75, 42 + cArmBufferY);
    public static final Point cMiddleCone = new Point(20 + cArmBufferX, 35 + cArmBufferY);
    public static final Point cMiddleCube = new Point(22.75, 28 + cArmBufferY);
    public static final Point cHomeStageOne = new Point(26.75, 28 + cArmBufferY);
    public static final Point cShelfPickup = new Point(22.75 + cArmBufferX, 34 + cArmBufferY);

    public static final Point cGroundPickup = new Point(4, 0);
    public static final Point cGroundPickup2 = new Point(7, 0);
    public static final Point cGroundDropoff = new Point(6, 9);
    public static final Point cArmOrigin = new Point(-17, 10);

    public static final int cCompressor = 11;
    public static final int cSolenoidFWD = 4;
    public static final int cSolenoidREV = 7;

    public static final double[] cCompetitionOffsets = {
            245.6,
            319.5,
            164,
            276.4
    };

    public static final double[] cTestOffsets = {
            47.81249999999999 + 180,
            47.900390625,
            193.447265625,
            202.41210937500003 + 180
    };

    public static final SwerveDriveKinematics cCompetitionKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(0.24765, 0.284),
                    new Translation2d(0.24765, -0.284),
                    new Translation2d(-0.24765, 0.284),
                    new Translation2d(-0.24765, -0.284)
            );

    public static final SwerveDriveKinematics cTestKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(0.2508, 0.284),
                    new Translation2d(0.2508, -0.284),
                    new Translation2d(-0.2508, 0.284),
                    new Translation2d(-0.2508, -0.284)
            );

    public static final String cCompetitionSerialNumber = "";

    public static final String cTestSerialNumber = "031c007a";

    public static double[] cYCoordDrop = new double[]{0.5, 1.0, 1.6, 2.2, 2.75, 3.3, 3.85, 4.4, 4.95};

    public static int[] cCompetitionSpeedControllers = {6, 5, 4, 3, 2, 1, 8, 7};

    public static int[] cTestSpeedControllers = {2, 1, 4, 3, 6, 5, 8, 7};

    public static double rampRate = .5;

    public static double llOffsetX = 0.1959;
    public static double llOffsetY = -0.28;
    public static double resHeight = 480;
    public static double yOffset = 0;
    public static double xOffset = 0.07;

}
