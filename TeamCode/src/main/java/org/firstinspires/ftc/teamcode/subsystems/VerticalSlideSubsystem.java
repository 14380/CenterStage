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

    private int POS_1_HEIGHT = 800;

    private int POS_2_HEIGHT = 1400;

    public VerticalSlideSubsystem(HardwareMap map, RobotStateSubsystem state){
        verticalMotor = map.get(DcMotorEx.class, "verticalSlide");
        verticalMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Extend(){
        verticalMotor.setTargetPosition(EXTENDED_POS + 20);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(1);
    }

    public void Retract(){
        verticalMotor.setTargetPosition(RETRACT_POS);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(1);
    }

    public void Position1(){
        verticalMotor.setTargetPosition(POS_1_HEIGHT);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(1);
    }

    public boolean IsPosition1(){

        return verticalMotor.getCurrentPosition() >= (POS_1_HEIGHT - 20);
    }

    public void Position2(){
        verticalMotor.setTargetPosition(POS_2_HEIGHT);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(1);
    }

    public boolean IsPosition2(){
        return verticalMotor.getCurrentPosition() >= (POS_2_HEIGHT - 20);
    }

    public boolean IsExtended(){
        return verticalMotor.getCurrentPosition() > EXTENDED_POS;
    }

    public int getCurrentPosition(){
        return verticalMotor.getCurrentPosition();
    }

    public boolean IsRetracted(){
        return verticalMotor.getCurrentPosition() <= (RETRACT_POS + 25);
    }
}

