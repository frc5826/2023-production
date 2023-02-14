package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.fabrik.Point;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PID;

import static frc.robot.Constants.*;

public class ArmPresetGoalCommand extends CommandBase {

    private PID mastPID, armPID;

    private Point goal;

    private double mastGoalRad, armGoalRad;

    private ArmSubsystem armSubsystem;

    public ArmPresetGoalCommand(ArmSubsystem armSubsystem, Point goal){
        this.armSubsystem = armSubsystem;
        this.goal = new Point();
        this.goal.setCoords(goal);
        this.mastPID = new PID(cMastP, cMastI, cMastD, 0.4, 0.1, 0.05);
        this.armPID = new PID(cArmP, cArmI, cArmD, 0.4, 0.1, 0.05);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setArmGoal(goal);
        armSubsystem.calculateArm();

        mastGoalRad = armSubsystem.getCalculatedMastAngle();
        armGoalRad = armSubsystem.getCalculatedArmAngle();

        mastPID.setGoal(mastGoalRad);
        armPID.setGoal(armGoalRad);
    }

    @Override
    public void execute() {
        armSubsystem.setMastSpeed(armPID.calculate(armSubsystem.getMastRad()));
        armSubsystem.setArmSpeed(armPID.calculate(armSubsystem.getArmRad()));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(armSubsystem.getMastRad() - mastGoalRad) <= 0.05 && Math.abs(armSubsystem.getArmRad() - armGoalRad) <= 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
