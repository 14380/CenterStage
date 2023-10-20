package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WinchSubsystem extends SubsystemBase {
    private DcMotorEx winchMotor;

    public WinchSubsystem(HardwareMap map){
        winchMotor = map.get(DcMotorEx.class, "winch");
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
}
