// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    ShuffleboardTab commandTab = Shuffleboard.getTab("Commands");

    SendableChooser<Command> comboBox = new SendableChooser<Command>();

    ArmRepositionCommand moveMastFwdCommand = new ArmRepositionCommand(armSubsystem, -10, 0);
    ArmRepositionCommand moveMastBkwCommand = new ArmRepositionCommand(armSubsystem, 10, 0);
    ArmRepositionCommand moveArmFwdCommand = new ArmRepositionCommand(armSubsystem, 0, 15);
    ArmRepositionCommand moveArmBkwCommand = new ArmRepositionCommand(armSubsystem, 0, -15);

    ArmPresetPositionCommand topCubeCommand = new ArmPresetPositionCommand(armSubsystem, cTopCube);
    ArmPresetPositionCommand topConeCommand = new ArmPresetPositionCommand(armSubsystem, cTopCone);
    ArmPresetPositionCommand middleCubeCommand = new ArmPresetPositionCommand(armSubsystem, cMiddleCube);
    ArmPresetPositionCommand middleConeCommand = new ArmPresetPositionCommand(armSubsystem, cMiddleCone);
    ArmPresetPositionCommand groundPickupCommand = new ArmPresetPositionCommand(armSubsystem, cGroundPickup);
    ArmPresetPositionCommand groundDropoffCommand = new ArmPresetPositionCommand(armSubsystem, cGroundDropoff);

    {
        if (System.getenv("serialnum").equals(cTestSerialNumber)) {
            driveSubsystem = new DriveSubsystem(cTestKinematics, cTestOffsets, cTestSpeedControllers);
        } else {
            driveSubsystem = new DriveSubsystem(cCompetitionKinematics, cCompetitionOffsets, cCompetitionSpeedControllers);
        }
    }

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

        comboBox.addOption("Zero Gyro", new InstantCommand(driveSubsystem::zeroGyroYaw));
        comboBox.addOption("Invert Gyro", new InstantCommand(driveSubsystem::invertGyroYaw));

        commandTab.add("Command Chooser", comboBox);

        comboBox.getSelected();

        // Configure the trigger bindings
        configureBindings();
        CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, fieldOrientedDriveCommand);

        camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240);
        camera.setFPS(20);
    }
    
    
    /** Use this method to define your trigger->command mappings. */
    private void configureBindings() {
        button3.onTrue(moveMastBkwCommand);
        button4.onTrue(moveArmBkwCommand);
        button5.onTrue(moveMastFwdCommand);
        button6.onTrue(moveArmFwdCommand);

        PanelButtons[0].onTrue(groundPickupCommand);
        PanelButtons[1].onTrue(groundDropoffCommand);
        PanelButtons[2].onTrue(topConeCommand);
        PanelButtons[3].onTrue(middleConeCommand);
        PanelButtons[4].whileTrue(autoAlignConeCommand);
        PanelButtons[7].onTrue(grabbinCommand);
        PanelButtons[8].onTrue(topCubeCommand);
        PanelButtons[9].onTrue(middleCubeCommand);
        PanelButtons[10].whileTrue(autoAlignCubeCommand);


        commandTab.add("Top Cube Command", topCubeCommand);
        commandTab.add("Top Cone Command", topConeCommand);
        commandTab.add("Middle Cube Command", middleCubeCommand);
        commandTab.add("Middle Cone Command", middleConeCommand);
        commandTab.add("Ground Pickup Command", groundPickupCommand);
        commandTab.add("Ground Dropoff Command", groundDropoffCommand);

        zeroGyroJoystick.onTrue(new InstantCommand(() -> {
            driveSubsystem.zeroGyroYaw();
            driveSubsystem.zeroGyroRollPitch();
        }));

        zeroGyroXbox.onTrue(new InstantCommand(() -> {
            driveSubsystem.zeroGyroYaw();
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
        //PathPlannerTrajectory path = PathPlanner.loadPath("New Path", 1, 1);

        //HashMap<String, Command> autoEventMap = new HashMap<>();
//        autoEventMap.put("marker1", new PrintCommand("Passed marker 1"));
//        autoEventMap.put("marker2", new PrintCommand("Passed marker 2"));
//
//        return new FollowPathWithEvents(
//            driveSubsystem.followTrajectoryCommand(path),
//            path.getMarkers(),
//            autoEventMap
//        );

        return comboBox.getSelected();
    }
}
