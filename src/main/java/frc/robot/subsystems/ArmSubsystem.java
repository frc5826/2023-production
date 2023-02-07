package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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

    private DutyCycleEncoder mastEncoder, armEncoder;

    public ArmSubsystem(){

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Arm");

        mastMotor = new WPI_TalonSRX(cBaseArmID);
        mastMotor.setNeutralMode(NeutralMode.Brake);

        mastMotor.configPeakOutputForward(0.4, cTimeoutMs);
        mastMotor.configPeakOutputReverse(-0.4, cTimeoutMs);

        armMotor = new WPI_TalonSRX(cMiddleArmID);
        armMotor.setNeutralMode(NeutralMode.Brake);

        armMotor.configPeakOutputForward(0.4, cTimeoutMs);
        armMotor.configPeakOutputReverse(-0.4, cTimeoutMs);


        mastEncoder = new DutyCycleEncoder(9);
        armEncoder = new DutyCycleEncoder(8);

        shuffleboardTab.addNumber("Mast Encoder Degrees", this::getMastDeg);
        shuffleboardTab.addNumber("Arm Encoder Degrees", this::getArmDeg);

        shuffleboardTab.addNumber("Mast Encoder Absolute", this::getMastPosition);
        shuffleboardTab.addNumber("Arm Encoder Absolute", this::getArmPosition);

    }

    public double getMastRad(){
        return (getMastPosition() - cMastEncoderOffset) * (Math.PI / cArmClicksPerRotation);
    }

    public double getArmRad(){
        return (getArmPosition() - cArmEncoderOffset) * (Math.PI / cArmClicksPerRotation);
    }

    public double getMastDeg(){
        return (getMastPosition() - cMastEncoderOffset) * (360 / cArmClicksPerRotation);
    }

    public double getArmDeg(){
        return (getArmPosition() - cArmEncoderOffset) * (360 / cArmClicksPerRotation);
    }

    public void setMastRad(double position){
        mastMotor.set(ControlMode.Position, Math.min(Math.max(position * (cArmClicksPerRotation /Math.PI) + cMastEncoderOffset, cMastEncoderMin), cMastEncoderMax));
    }

    public void setArmRad(double position){
        armMotor.set(ControlMode.Position, Math.min(Math.max(position * (cArmClicksPerRotation /Math.PI) + cArmEncoderOffset, cArmEncoderMin), cArmEncoderMax));
    }

    public void setMastSpeed(double speed){
        mastMotor.set(speed);
    }

    public void setArmSpeed(double speed){
        armMotor.set(speed);
    }

    public void setMastPosition(double position){
        mastMotor.set(ControlMode.Position, position);
    }

    public void setArmPosition(double position){
        armMotor.set(ControlMode.Position, position);
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

}
