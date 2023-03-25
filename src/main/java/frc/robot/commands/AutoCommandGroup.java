package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoCommandGroup extends SequentialCommandGroup {
    public AutoCommandGroup(Command setupGyroCommand, Command topCube, Command autoAlign, Command drop, Command pathPlanner) {
        addCommands(
              setupGyroCommand,
              topCube,
              new WaitCommand(1.6),
              autoAlign,
              drop,
              pathPlanner
        );
    }
}
