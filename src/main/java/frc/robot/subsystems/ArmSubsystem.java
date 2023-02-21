package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.fabrik.Arm2Segment;
import frc.robot.fabrik.Point;

import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    Arm2Segment arm = new Arm2Segment(cArmLengths, 0.5, cArmOrigin);

    private WPI_TalonSRX mastMotor, armMotor;
    private PIDController mastPID, armPID;
    private DutyCycleEncoder mastEncoder, armEncoder;
    private double armSpeed, mastSpeed;
    private double armTargetAngle, mastTargetAngle;

    public ArmSubsystem(){

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Arm");

        mastMotor = new WPI_TalonSRX(cBaseArmID);
        mastMotor.setNeutralMode(NeutralMode.Brake);

        mastMotor.configPeakOutputForward(0.4, cTimeoutMs);
        mastMotor.configPeakOutputReverse(-0.4, cTimeoutMs);

        armMotor = new WPI_TalonSRX(cMiddleArmID);
        armMotor.setNeutralMode(NeutralMode.Brake);

        armMotor.configPeakOutputForward(1, cTimeoutMs);
        armMotor.configPeakOutputReverse(-1, cTimeoutMs);

        mastEncoder = new DutyCycleEncoder(cMastEncoderID);
        armEncoder = new DutyCycleEncoder(cArmEncoderID);

        mastPID = new PIDController(cMastP, cMastI, cMastD);
        //armPID = new PIDController(cArmP, cArmI, cArmD);
        armPID = new PIDController(3, 0, 0);

        setMastTargetRad(cMastEncoderMax);
        setArmTargetRad(cArmEncoderMin);


        shuffleboardTab.addNumber("Mast PID Target", this::getMastTargetAngle).withSize(1, 1).withPosition(5, 0);
        shuffleboardTab.addNumber("Arm PID Target", this::getArmTargetAngle).withSize(1, 1).withPosition(5, 1);

        shuffleboardTab.addNumber("Mast Motor Speed", this::getMastSpeed).withSize(1, 1).withPosition(6, 0);
        shuffleboardTab.addNumber("Arm Motor Speed", this::getArmSpeed).withSize(1, 1).withPosition(6, 1);

        shuffleboardTab.addNumber("Mast Error", mastPID::getPositionError).withSize(1, 1).withPosition(4, 0);
        shuffleboardTab.addNumber("Arm Error", armPID::getPositionError).withSize(1, 1).withPosition(4, 1);

        shuffleboardTab.add("Mast PID", mastPID);
        shuffleboardTab.add("Arm PID", armPID);

        shuffleboardTab.addNumber("Mast Encoder Degrees", this::getMastDeg);
        shuffleboardTab.addNumber("Arm Encoder Degrees", this::getArmDeg);

        shuffleboardTab.addNumber("Mast Encoder Absolute", this::getMastPosition);
        shuffleboardTab.addNumber("Arm Encoder Absolute", this::getArmPosition);

    }

    @Override
    public void periodic() {
        super.periodic();
        mastSpeed = mastPID.calculate(getMastRad());
        armSpeed = armPID.calculate(getArmRad());

        setMastSpeed(mastSpeed);
        setArmSpeed(armSpeed);
    }

    public void setMastTargetRad(double setpoint){
        mastTargetAngle = Math.max(Math.min(setpoint, cMastEncoderMax), cMastEncoderMin);
        System.out.println("Mast target set to " + mastTargetAngle);
        mastPID.setSetpoint(mastTargetAngle);
    }

    public void setArmTargetRad(double setpoint){
        armTargetAngle = Math.max(Math.min(setpoint, cArmEncoderMax), cArmEncoderMin);
        System.out.println("Arm target set to " + armTargetAngle);
        armPID.setSetpoint(armTargetAngle);
    }

    public double getMastRad(){
        return (getMastPosition() - cMastEncoderOffset) * ((Math.PI * 2) / cArmClicksPerRotation);
    }

    public double getArmRad(){
        return (getArmPosition() - cArmEncoderOffset) * ((Math.PI * 2) / cArmClicksPerRotation);
    }

    public double getMastDeg(){
        return (getMastPosition() - cMastEncoderOffset) * (360 / cArmClicksPerRotation);
    }

    public double getArmDeg(){
        return (getArmPosition() - cArmEncoderOffset) * (360 / cArmClicksPerRotation);
    }

    public void setMastSpeed(double speed){
        mastMotor.set(Math.min(Math.max(speed, -cMastMaxSpeed), cMastMaxSpeed));
    }

    public void setArmSpeed(double speed){
        armMotor.set(Math.min(Math.max(speed, -cArmMaxSpeed), cArmMaxSpeed));
    }

    public double getMastPosition(){
        return mastEncoder.getAbsolutePosition();
    }

    public double getArmPosition(){
        return armEncoder.getAbsolutePosition();
    }

    public void setArmGoal(Point goal){
        arm.setGoal(goal);
    }

    public void calculateArm(){
        arm.fabrik();
    }

    public double getCalculatedMastAngle(){
        return arm.getArmBaseAngle();
    }

    public double getCalculatedArmAngle(){
        return arm.getArmMiddleAngle();
    }

    public double getMastTargetAngle(){
        return  mastTargetAngle * (180/Math.PI);
    }

    public double getArmTargetAngle() {
        return armTargetAngle * (180/Math.PI);
    }

    public double getMastSpeed() {
        return mastSpeed;
    }

    public double getArmSpeed() {
        return armSpeed;
    }

}
