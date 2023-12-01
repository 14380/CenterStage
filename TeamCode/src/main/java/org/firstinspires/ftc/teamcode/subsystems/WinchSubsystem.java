package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class WinchSubsystem extends SubsystemBase {
    private DcMotorEx winchMotor;

    private ServoImplEx leftDeployServo;
    private ServoImplEx rightDeployServo;

    public WinchSubsystem(HardwareMap map, RobotStateSubsystem state)
    {
        winchMotor = map.get(DcMotorEx.class, "winch");
        leftDeployServo = map.get(ServoImplEx.class, "leftDeploy");
        rightDeployServo = map.get(ServoImplEx.class, "rightDeploy");

        //rightDeployServo.setDirection(Servo.Direction.REVERSE);
        leftDeployServo.setDirection(Servo.Direction.REVERSE);
        leftDeployServo.setPosition(0);
        rightDeployServo.setPosition(0);
    }

    public void WinchOn(){
        winchMotor.setPower(1);
    }
    public void WinchOff(){
        winchMotor.setPower(0);
    }
    public void WinchReverse(){
        winchMotor.setPower(-1);
    }

    public void DeployLeftHooks()
    {
        leftDeployServo.setPosition(0.4);
    }

    public void DeployRightHooks()
    {
        rightDeployServo.setPosition(0.4);
    }
}
