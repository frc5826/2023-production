package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoCommandGroup extends SequentialCommandGroup {
    public AutoCommandGroup(Command topCube, Command autoAlign, Command drop, Command pathPlanner) {
        addCommands(
              topCube,
              autoAlign,
              drop,
              pathPlanner
        );
    }
}
