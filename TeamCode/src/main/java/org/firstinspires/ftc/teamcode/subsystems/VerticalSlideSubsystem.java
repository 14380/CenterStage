package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VerticalSlideSubsystem extends SubsystemBase {

    private DcMotorEx verticalMotor;

    private int EXTENDED_POS = 1750;
    private int RETRACT_POS = 10;

    public VerticalSlideSubsystem(HardwareMap map){
        verticalMotor = map.get(DcMotorEx.class, "verticalSlide");
        verticalMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void Extend(){
        verticalMotor.setTargetPosition(EXTENDED_POS + 20);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(1);
    }

    public void Retract(){
        verticalMotor.setTargetPosition(RETRACT_POS - 20);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(1);
    }

    public boolean IsExtended(){
        return verticalMotor.getCurrentPosition() > EXTENDED_POS;
    }

    public int getCurrentPosition(){
        return verticalMotor.getCurrentPosition();
    }

    public boolean IsRetracted(){
        return verticalMotor.getCurrentPosition() < RETRACT_POS;
    }
}

