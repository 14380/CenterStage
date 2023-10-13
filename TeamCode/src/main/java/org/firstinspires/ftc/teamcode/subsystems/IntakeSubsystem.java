package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {

    private DcMotorEx intakeMotor;

    public IntakeSubsystem(HardwareMap map){
        intakeMotor = map.get(DcMotorEx.class, "intake");

    }

    public void IntakeOn(){
        intakeMotor.setPower(1);
    }

    public void IntakeOff(){
        intakeMotor.setPower(0);
    }

    public void Reverse(){
        intakeMotor.setPower(-1);
    }
}
