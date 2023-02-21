package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
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

    public static final Trigger[] PanelButtons;

    static {
        PanelButtons = new Trigger[11];

        for(int i = 0; i<11; i++) {
            int finalI = i+1;
            PanelButtons[i] = new Trigger(() -> cButtonPanel.getRawButton(finalI));
        }
    }

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
    public static final double[] cArmLengths = {41, 46};

    //TODO find important encoder values
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


    //TODO separate arm segment PID values
    public static final double cMastP = 10;
    public static final double cMastI = 0.5;
    public static final double cMastD = 0;
    public static final double cMastMaxSpeed = 0.4;
    public static final double cArmP = 3;
    public static final double cArmI = 0;
    public static final double cArmD = 0;
    public static final double cArmMaxSpeed = 0.4;

    public static final double cDriveSpeed = 5;
    public static final double cTurnSpeed = 1.5;

    public static Trigger zeroGyroXbox = new Trigger(() -> cXbox.getBButtonPressed());

    public static Trigger vibrateXbox = new Trigger(() -> cXbox.getRightBumperPressed());

    public static Trigger zeroGyroJoystick = new Trigger(() -> cJoystick.getRawButtonPressed(8));

    public static Trigger autoBalance = new Trigger(() -> cJoystick.getRawButton(11));
    public static Trigger align = new Trigger(() -> cXbox.getAButton());
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
    public static final Point cTopCube = new Point(39.75, 40 + cArmBufferY);
    public static final Point cMiddleCone = new Point(22.75 + cArmBufferX, 34 + cArmBufferY);
    public static final Point cMiddleCube = new Point(22.75, 28 + cArmBufferY);

    public static final Point cGroundPickup = new Point(5, 2);
    public static final Point cGroundDropoff = new Point(6, 9);
    public static final Point cArmOrigin = new Point(-17, 10);

    public static final int cCompressor = 11;
    public static final int cSolenoidFWD = 4;
    public static final int cSolenoidREV = 7;

    public static final double[] cCompetitionOffsets = {
            66.248149628112955 + 180,
            250.7614831463437 + 180,
            307.4751996023543 + 180,
            333.896484375 + 180
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

    public static double[] yCoordDrop = new double[]{0.5, 1.0, 1.6, 2.2, 2.75, 3.3, 3.85, 4.4, 4.95};

    public static int[] cCompetitionSpeedControllers = {6, 5, 4, 3, 2, 1, 8, 7};

    public static int[] cTestSpeedControllers = {2, 1, 4, 3, 6, 5, 8, 7};

}
