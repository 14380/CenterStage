package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizontalSlideSubsystem extends SubsystemBase {

    private DcMotorEx horizontalMotor;

    private int EXTENDED_POS = 2100;
    private int RETRACT_POS = 50;

    public HorizontalSlideSubsystem(HardwareMap map){
        horizontalMotor = map.get(DcMotorEx.class, "horizontalSlide");
        horizontalMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Extend(){
        horizontalMotor.setTargetPosition(EXTENDED_POS + 20);
        horizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalMotor.setPower(1);
    }

    public void Retract(){
        horizontalMotor.setTargetPosition(RETRACT_POS - 20);
        horizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalMotor.setPower(1);
    }

    public boolean IsExtended(){
        return horizontalMotor.getCurrentPosition() > EXTENDED_POS;
    }

    public int getCurrentPosition(){
        return horizontalMotor.getCurrentPosition();
    }

    public boolean IsRetracted(){
        return horizontalMotor.getCurrentPosition() < RETRACT_POS;
    }
}
