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
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.ArmLockCommand;
import frc.robot.commands.ArmMoveCommand;
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

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    ArmSubsystem armSubsystem = new ArmSubsystem();
    VisionSubsystem visionSubsystem = new VisionSubsystem();

    ArmMoveCommand armMoveCommand = new ArmMoveCommand(armSubsystem);
    ArmLockCommand armLockCommand = new ArmLockCommand(armSubsystem);
    FieldOrientedDriveCommand fieldOrientedDriveCommand = new FieldOrientedDriveCommand(driveSubsystem);
    AutoBalanceCommand autoBalanceCommand = new AutoBalanceCommand(driveSubsystem);
    AutoAlignCommand autoAlignCubeCommand = new AutoAlignCommand(driveSubsystem, visionSubsystem, true);
    AutoAlignCommand autoAlignConeCommand = new AutoAlignCommand(driveSubsystem, visionSubsystem, false);

    GrabbinSubsystem grabbinSubsystem = new GrabbinSubsystem();
    GrabbinCommand grabbinCommand = new GrabbinCommand(grabbinSubsystem);

    JoystickButton trigger = new JoystickButton(cJoystick, 1);

    private UsbCamera camera;

    public RobotContainer()
    {

        //TODO can use System.getenv("serialnum") to get the rio, practice robot: 031c007a

        // Configure the trigger bindings
        configureBindings();
        CommandScheduler.getInstance().setDefaultCommand(armSubsystem, armMoveCommand);
        CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, fieldOrientedDriveCommand);

        camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240);
        camera.setFPS(20);
    }
    
    
    /** Use this method to define your trigger->command mappings. */
    private void configureBindings()
    {
        trigger.whileTrue(armLockCommand);

        zeroGyroJoystick.onTrue(new InstantCommand(() -> {
            driveSubsystem.zeroGyroYaw();
            driveSubsystem.zeroGyroRollPitch();
        }));

        zeroGyroXbox.onTrue(new InstantCommand(() -> {
            driveSubsystem.zeroGyroYaw();
            driveSubsystem.zeroGyroRollPitch();
        }));

        Constants.autoBalance.whileTrue(autoBalanceCommand);

        PanelButtons[2].whileTrue(new FunctionalCommand(() -> cXbox.setRumble(GenericHID.RumbleType.kBothRumble, 1), () -> {}, (Boolean on) -> cXbox.setRumble(GenericHID.RumbleType.kBothRumble, 0), () -> false));

        PanelButtons[0].whileTrue(autoAlignCubeCommand);
        PanelButtons[1].whileTrue(autoAlignConeCommand);
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        PathPlannerTrajectory path = PathPlanner.loadPath("New Path", 1, 1);

        HashMap<String, Command> autoEventMap = new HashMap<>();
        autoEventMap.put("marker1", new PrintCommand("Passed marker 1"));
        autoEventMap.put("marker2", new PrintCommand("Passed marker 2"));

        return new FollowPathWithEvents(
            driveSubsystem.followTrajectoryCommand(path),
            path.getMarkers(),
            autoEventMap
        );
    }
}
