// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
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
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    ArmSubsystem armSubsystem = new ArmSubsystem();
    VisionSubsystem visionSubsystem = new VisionSubsystem();

    ShuffleboardTab commandTab = Shuffleboard.getTab("Commands");

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

    FieldOrientedDriveCommand fieldOrientedDriveCommand = new FieldOrientedDriveCommand(driveSubsystem);
    AutoBalanceCommand autoBalanceCommand = new AutoBalanceCommand(driveSubsystem);
    AutoAlignCommand autoAlignCommand = new AutoAlignCommand(driveSubsystem, visionSubsystem);

    GrabbinSubsystem grabbinSubsystem = new GrabbinSubsystem();
    GrabbinCommand grabbinCommand = new GrabbinCommand(grabbinSubsystem);

    JoystickButton trigger = new JoystickButton(cJoystick, 1);
    JoystickButton button3 = new JoystickButton(cJoystick, 3);
    JoystickButton button4 = new JoystickButton(cJoystick, 4);
    JoystickButton button5 = new JoystickButton(cJoystick, 5);
    JoystickButton button6 = new JoystickButton(cJoystick, 6);
    JoystickButton button10 = new JoystickButton(cJoystick, 10);

    public RobotContainer()
    {
        Shuffleboard.getTab("Arm").add(powerDistribution);

        // Configure the trigger bindings
        configureBindings();
        CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, fieldOrientedDriveCommand);
    }
    
    
    /** Use this method to define your trigger->command mappings. */
    private void configureBindings()
    {
        trigger.onTrue(grabbinCommand);
        button3.onTrue(moveMastBkwCommand);
        button4.onTrue(moveArmBkwCommand);
        button5.onTrue(moveMastFwdCommand);
        button6.onTrue(moveArmFwdCommand);

        button10.onTrue(topCubeCommand);

        commandTab.add("Top Cube Command", topCubeCommand);
        commandTab.add("Top Cone Command", topConeCommand);
        commandTab.add("Middle Cube Command", middleCubeCommand);
        commandTab.add("Middle Cone Command", middleConeCommand);
        commandTab.add("Ground Pickup Command", groundPickupCommand);
        commandTab.add("Ground Dropoff Command", groundDropoffCommand);

        Constants.zeroGyro.onTrue(new InstantCommand(() -> {
            driveSubsystem.zeroGyroYaw();
            driveSubsystem.zeroGyroRollPitch();
        }));

        Constants.autoBalance.whileTrue(autoBalanceCommand);

        Constants.align.whileTrue(autoAlignCommand);
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // TODO: Implement properly
        return driveSubsystem.followTrajectoryCommand(PathPlanner.loadPath("New Path", 1, 1));
    }
}
