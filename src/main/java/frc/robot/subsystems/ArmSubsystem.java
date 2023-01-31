package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private WPI_TalonSRX armBaseMotor, armMiddleMotor;

    public ArmSubsystem(){

        armBaseMotor = new WPI_TalonSRX(cBaseArmID);
        armBaseMotor.setNeutralMode(NeutralMode.Brake);

        armBaseMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, cArmPIDLoopID, cTimeoutMs);

        armBaseMotor.config_kF(cArmPIDLoopID, 0, cTimeoutMs);
        armBaseMotor.config_kP(cArmPIDLoopID, cArmP, cTimeoutMs);
        armBaseMotor.config_kI(cArmPIDLoopID, cArmI, cTimeoutMs);
        armBaseMotor.config_kD(cArmPIDLoopID, cArmD, cTimeoutMs);

        armBaseMotor.configPeakOutputForward(0.4, cTimeoutMs);
        armBaseMotor.configPeakOutputReverse(-0.4, cTimeoutMs);



        armMiddleMotor = new WPI_TalonSRX(cMiddleArmID);
        armMiddleMotor.setNeutralMode(NeutralMode.Brake);

        armMiddleMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, cArmPIDLoopID, cTimeoutMs);

        armMiddleMotor.config_kF(cArmPIDLoopID, 0, cTimeoutMs);
        armMiddleMotor.config_kP(cArmPIDLoopID, cArmP, cTimeoutMs);
        armMiddleMotor.config_kI(cArmPIDLoopID, cArmI, cTimeoutMs);
        armMiddleMotor.config_kD(cArmPIDLoopID, cArmD, cTimeoutMs);

        armMiddleMotor.configPeakOutputForward(0.4, cTimeoutMs);
        armMiddleMotor.configPeakOutputReverse(-0.4, cTimeoutMs);

    }

    public void setBaseSpeed(double speed){
        armBaseMotor.set(speed);
    }

    public void setMiddleSpeed(double speed){
        armMiddleMotor.set(speed);
    }

    public void setBasePosition(double position){
        armBaseMotor.set(ControlMode.Position, position);
    }

    public void setMiddlePosition(double position){
        armMiddleMotor.set(ControlMode.Position, position);
    }

    public double getBaseEncoderPosition(){
        return armBaseMotor.getSelectedSensorPosition(cArmPIDLoopID);
    }

    public double getMiddleEncoderPosition(){
        return armMiddleMotor.getSelectedSensorPosition(cArmPIDLoopID);
    }

}
