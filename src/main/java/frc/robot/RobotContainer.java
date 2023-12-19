// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.enums.ArmControlMode;
import frc.robot.enums.GrabType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.HashMap;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    PowerDistribution powerDistribution = new PowerDistribution(21, PowerDistribution.ModuleType.kCTRE);
    private final DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem = new VisionSubsystem();

    ShuffleboardTab dashboard = Shuffleboard.getTab("Driving");

    SendableChooser<Command> comboBox = new SendableChooser<Command>();

    {
        if (System.getenv("serialnum").equals(cTestSerialNumber)) {
            driveSubsystem = new DriveSubsystem(cTestKinematics, cTestOffsets, cTestSpeedControllers);
        } else {
            driveSubsystem = new DriveSubsystem(cCompetitionKinematics, cCompetitionOffsets, cCompetitionSpeedControllers);
        }
    }

    SwerveAutoBuilder autoBuilder;

    FieldOrientedDriveCommand fieldOrientedDriveCommand = new FieldOrientedDriveCommand(driveSubsystem);

    //SequentialCommandGroup homeArm = new SequentialCommandGroup(new GrabbinCommand(grabbinSubsystem, GrabType.CLOSE), homeStageOneCommand, new MastWaitCommand(armSubsystem, 0.7, 45),  new ArmPresetPositionCommand(armSubsystem));

//    JoystickButton trigger = new JoystickButton(cJoystick, 1);
//    JoystickButton button3 = new JoystickButton(cJoystick, 3);
//    JoystickButton button4 = new JoystickButton(cJoystick, 4);
//    JoystickButton button5 = new JoystickButton(cJoystick, 5);
//    JoystickButton button6 = new JoystickButton(cJoystick, 6);
//    JoystickButton button10 = new JoystickButton(cJoystick, 10);

    private UsbCamera camera;

    public RobotContainer()
    {
        Shuffleboard.getTab("Arm").add(powerDistribution);

        autoBuilder = driveSubsystem.getAutoBuilder(eventMap());
        //comboBox.addOption("test Start", testStart());

        comboBox.setDefaultOption("No exit (any)", new InstantCommand());

        comboBox.addOption("Top red (3)", autoPath("Top start red", 1.5f));
        comboBox.addOption("Center red (2)", autoPath("Center start red", 0.9f));
        comboBox.addOption("Bottom red (1)", autoPath("Bottom start red", 1.5f));

        comboBox.addOption("Top blue (6)", autoPath("Top start blue ", 1.5f));
        comboBox.addOption("Center blue (7)", autoPath("Center start blue", 0.9f));
        comboBox.addOption("Bottom blue (8)", autoPath("Bottom start blue", 1.5f));

        comboBox.addOption("Center blue no x (7)", autoPath("Center start blue old", 0.9f));
        comboBox.addOption("Center red no x (2)", autoPath("Center start red old", 0.9f));

        comboBox.addOption("Test path", autoPath("Test path", 0.8f));

        dashboard.add("Command Chooser", comboBox).withSize(2,2).withPosition(7, 0);

        dashboard.addNumber("Drive speed", () -> Constants.cDriveSpeed);

//        dashboard.add(new SequentialCommandGroup(
//                new InstantCommand(() ->
//                {driveSubsystem.resetOdometryButCool(new double[]{2.2, 2.75});
//                driveSubsystem.zeroGyroYaw();}),
//                autoPath("New Path", 1.5f)));

        comboBox.getSelected();

        // Configure the trigger bindings
        configureBindings();
        CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, fieldOrientedDriveCommand);

        camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240);
        camera.setFPS(10);
        try {
            dashboard.addCamera("Limelight", "limelight-avis", "10.58.26.11:5800").withSize(3, 3).withPosition(3, 0);
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            dashboard.add(camera).withSize(3, 3).withPosition(0, 0);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /** Use this method to define your trigger->command mappings. */
    private void configureBindings() {


//        zeroGyroJoystick.onTrue(new InstantCommand(() -> {
//            driveSubsystem.zeroGyroYaw();
//            driveSubsystem.zeroGyroRollPitch();
//        }));

        cXboxStart.onTrue(new InstantCommand(() -> {
            driveSubsystem.zeroDriveGyro();
            driveSubsystem.zeroGyroRollPitch();
        }));



//        vibrateXbox.onTrue(new FunctionalCommand(() -> cXbox.setRumble(GenericHID.RumbleType.kBothRumble, 1), () -> {
//        }, (Boolean on) -> cXbox.setRumble(GenericHID.RumbleType.kBothRumble, 0), () -> false));
//
//        Constants.autoBalance.whileTrue(autoBalanceCommand);
//
//        PanelButtons[4].whileTrue(new FunctionalCommand(() -> cXbox.setRumble(GenericHID.RumbleType.kBothRumble, 1), () -> {
//        }, (Boolean on) -> cXbox.setRumble(GenericHID.RumbleType.kBothRumble, 0), () -> false));
//
//
//        PanelButtons[2].whileTrue(new InstantCommand(driveSubsystem::invertGyroYaw));
//        PanelButtons[3].whileTrue(new InstantCommand(driveSubsystem::zeroGyroYaw));

    }




    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return null;
    }

    private Command autoPath(String path, float vel) {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup(path, vel, 2));
    }

    public HashMap<String, Command> eventMap() {
        HashMap<String, Command> eventMap = new HashMap<>();


        eventMap.put("waitTinyTime", new WaitCommand(0.1));
        eventMap.put("waitHalfSec", new WaitCommand(.5));
        eventMap.put("wait2sec", new WaitCommand(2));
        eventMap.put("wait3sec", new WaitCommand(3));
        eventMap.put("waitRaiseArm", new WaitCommand(1.6));



        return eventMap;
    }
}
