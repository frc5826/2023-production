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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.GrabbinSubsystem;

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
    ArmSubsystem armSubsystem = new ArmSubsystem();
    VisionSubsystem visionSubsystem = new VisionSubsystem();

    ShuffleboardTab dashboard = Shuffleboard.getTab("Driving");

    SendableChooser<Command> comboBox = new SendableChooser<Command>();

    ArmRepositionCommand moveMastFwdCommand = new ArmRepositionCommand(armSubsystem, -10, 0);
    ArmRepositionCommand moveMastBkwCommand = new ArmRepositionCommand(armSubsystem, 10, 0);
    ArmRepositionCommand moveArmFwdCommand = new ArmRepositionCommand(armSubsystem, 0, 15);
    ArmRepositionCommand moveArmBkwCommand = new ArmRepositionCommand(armSubsystem, 0, -15);

    ArmPresetPositionCommand topCubeCommand = new ArmPresetPositionCommand(armSubsystem, cTopCube);
    ArmPresetPositionCommand topConeCommand = new ArmPresetPositionCommand(armSubsystem, cTopCone);
    ArmPresetPositionCommand middleCubeCommand = new ArmPresetPositionCommand(armSubsystem, cMiddleCube);
    ArmPresetPositionCommand homeStageOneCommand = new ArmPresetPositionCommand(armSubsystem, cHomeStageOne);
    ArmPresetPositionCommand middleConeCommand = new ArmPresetPositionCommand(armSubsystem, cMiddleCone);
    ArmPresetPositionCommand groundPickupCommand = new ArmPresetPositionCommand(armSubsystem, cGroundPickup);
    ArmPresetPositionCommand groundDropoffCommand = new ArmPresetPositionCommand(armSubsystem, cGroundDropoff);
    ArmPresetPositionCommand shelfPickupCommand = new ArmPresetPositionCommand(armSubsystem, cShelfPickup);
    ArmPresetPositionCommand homeStageTwoCommand = new ArmPresetPositionCommand(armSubsystem);

    {
        if (System.getenv("serialnum").equals(cTestSerialNumber)) {
            driveSubsystem = new DriveSubsystem(cTestKinematics, cTestOffsets, cTestSpeedControllers);
        } else {
            driveSubsystem = new DriveSubsystem(cCompetitionKinematics, cCompetitionOffsets, cCompetitionSpeedControllers);
        }
    }

    SequentialCommandGroup homeArm = new SequentialCommandGroup(homeStageOneCommand, new WaitCommand(0.4), homeStageTwoCommand);
    SwerveAutoBuilder autoBuilder;

    FieldOrientedDriveCommand fieldOrientedDriveCommand = new FieldOrientedDriveCommand(driveSubsystem);
    AutoBalanceCommand autoBalanceCommand = new AutoBalanceCommand(driveSubsystem);
    AutoAlignCommand autoAlignCubeCommand = new AutoAlignCommand(driveSubsystem, visionSubsystem, true);
    AutoAlignCommand autoAlignConeCommand = new AutoAlignCommand(driveSubsystem, visionSubsystem, false);

    GrabbinSubsystem grabbinSubsystem = new GrabbinSubsystem();
    GrabbinCommand grabbinCommand = new GrabbinCommand(grabbinSubsystem);

    JoystickButton trigger = new JoystickButton(cJoystick, 1);
    JoystickButton button3 = new JoystickButton(cJoystick, 3);
    JoystickButton button4 = new JoystickButton(cJoystick, 4);
    JoystickButton button5 = new JoystickButton(cJoystick, 5);
    JoystickButton button6 = new JoystickButton(cJoystick, 6);
    JoystickButton button10 = new JoystickButton(cJoystick, 10);

    private UsbCamera camera;

    public RobotContainer()
    {
        Shuffleboard.getTab("Arm").add(powerDistribution);

        autoBuilder = driveSubsystem.getAutoBuilder(eventMap());
        //comboBox.addOption("test Start", testStart());

        comboBox.addOption("Top start red", autoPath("Top start red", 1.3f));
        comboBox.addOption("Center start red", autoPath("Center start red", 0.8f));
        comboBox.addOption("Bottom start red", autoPath("Bottom start red", 1.3f));

        comboBox.addOption("Top start blue", autoPath("Top start blue", 1.3f));
        comboBox.addOption("Center start blue", autoPath("Center start blue", 0.8f));
        comboBox.addOption("Bottom start blue", autoPath("Bottom start blue", 1.3f));

        dashboard.add("Command Chooser", comboBox).withSize(2,2).withPosition(7, 0);

        comboBox.getSelected();

        // Configure the trigger bindings
        configureBindings();
        CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, fieldOrientedDriveCommand);

        camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240);
        camera.setFPS(10);
        dashboard.add(camera).withSize(3, 3).withPosition(0, 0);
        dashboard.addCamera("Limelight", "limelight-avis", "10.58.26.11:5800").withSize(3,3).withPosition(3,0);

    }

    /** Use this method to define your trigger->command mappings. */
    private void configureBindings() {
        cXboxX.onTrue(moveMastBkwCommand);
        cXboxA.onTrue(moveArmBkwCommand);
        cXboxB.onTrue(moveMastFwdCommand);
        cXboxY.onTrue(moveArmFwdCommand);

        cPanelButtons[0].onTrue(groundPickupCommand);
        cPanelButtons[1].onTrue(groundDropoffCommand);
        cPanelButtons[2].onTrue(topConeCommand);
        cPanelButtons[3].onTrue(middleConeCommand);
        cPanelButtons[4].whileTrue(autoAlignConeCommand);
        cPanelButtons[5].onTrue(homeArm);
        cPanelButtons[7].onTrue(grabbinCommand);
        cPanelButtons[8].onTrue(topCubeCommand);
        cPanelButtons[9].onTrue(middleCubeCommand);
        cPanelButtons[10].whileTrue(autoAlignCubeCommand);
        cPanelButtons[11].onTrue(shelfPickupCommand);

        zeroGyroJoystick.onTrue(new InstantCommand(() -> {
            driveSubsystem.zeroGyroYaw();
            driveSubsystem.zeroGyroRollPitch();
        }));

        cXboxStart.onTrue(new InstantCommand(() -> {
            driveSubsystem.zeroGyroYaw();
            driveSubsystem.zeroGyroRollPitch();
        }));

        autoBalance.whileTrue(autoBalanceCommand);

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
        return new AutoCommandGroup(
                new SetupGyroCommand(driveSubsystem, visionSubsystem),
                new ArmPresetPositionCommand(armSubsystem, cTopCube),
                new AutoAlignCommand(driveSubsystem, visionSubsystem, true, 2.5),
                new GrabbinCommand(grabbinSubsystem),
                comboBox.getSelected());

    }

    private Command autoPath(String path, float vel) {
        return autoBuilder.fullAuto(PathPlanner.loadPathGroup(path, vel, 1));
    }

    public HashMap<String, Command> eventMap() {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("homeArm", new SequentialCommandGroup(new ArmPresetPositionCommand(armSubsystem, cHomeStageOne), new WaitCommand(0.4), new ArmPresetPositionCommand(armSubsystem), new WaitCommand(0)));
        eventMap.put("groundPickUp", groundPickupCommand);
        eventMap.put("grab", grabbinCommand);
        eventMap.put("autoAlignCube", autoAlignCubeCommand);
        eventMap.put("autoAlignCone", autoAlignConeCommand);
        eventMap.put("dropHighCone", topConeCommand);
        eventMap.put("autoBalance", new AutoBalanceCommand(driveSubsystem));

        eventMap.put("wait2sec", new WaitCommand(2));
        eventMap.put("wait3sec", new WaitCommand(3));

        return eventMap;
    }
}
