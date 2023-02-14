package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PID;

import static frc.robot.Constants.*;

public class ArmLockCommand extends CommandBase {

    private ArmSubsystem armSubsystem;
    private PID armPID, mastPID;
    private double mastLockAngle, armLockAngle;
    private double mastSpeed, armSpeed;

    public ArmLockCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

        mastPID = new PID(cMastP, cMastI, cMastD, 0.4, 0, 0);
        armPID = new PID(cArmP, cArmI, cArmD, 0.4, 0, 0);

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Arm");

        shuffleboardTab.addNumber("Mast PID Target", this::getMastLockAngle);
        shuffleboardTab.addNumber("Arm PID Target", this::getArmLockAngle);

        shuffleboardTab.addNumber("Mast Motor Speed", this::getMastSpeed);
        shuffleboardTab.addNumber("Arm Motor Speed", this::getArmSpeed);

        shuffleboardTab.addNumber("Mast Error", mastPID::getError);
        shuffleboardTab.addNumber("Arm Error", armPID::getError);
    }

    @Override
    public void initialize() {
        super.initialize();

        this.mastLockAngle = armSubsystem.getMastRad();
        this.armLockAngle = Math.PI / 5; //armSubsystem.getArmRad();

        mastPID.setGoal(mastLockAngle);
        armPID.setGoal(armLockAngle);

        System.out.println("Arm lock init");
    }

    @Override
    public void execute() {
        super.execute();

        mastSpeed = mastPID.calculate(armSubsystem.getMastRad());
        armSpeed = armPID.calculate(armSubsystem.getArmRad());

        armSubsystem.setMastSpeed(mastSpeed);
        armSubsystem.setArmSpeed(armSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMastSpeed(0);
        armSubsystem.setArmSpeed(0);
        System.out.println("Arm lock end");
        super.end(interrupted);
    }

    public double getMastLockAngle(){
        return  mastLockAngle * (180/Math.PI);
    }

    public double getArmLockAngle() {
        return armLockAngle * (180/Math.PI);
    }

    public double getMastSpeed() {
        return mastSpeed;
    }

    public double getArmSpeed() {
        return armSpeed;
    }
}