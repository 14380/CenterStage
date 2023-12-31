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

    //position 1 of the vertical slides.
    private int POS_1_HEIGHT = 800;

    //a little higher than pos1
    private int AUTO_HEIGHT = 1050;


    //Extra height for second pixel in auto
    private int POS_AUTO_EX_HEIGHT = 1000;

    //this is the height of position 2 of the vertical slides.
    //This CAN NOT GO HIGHER than 1750 - EXTENDED_POS
    private int POS_2_HEIGHT = 1600;

    //first position of the slide lift.
    private int POS_0_HEIGHT = 300;

    private int POS_A_HEIGHT = 450;

    public VerticalSlideSubsystem(HardwareMap map, RobotStateSubsystem state){
        verticalMotor = map.get(DcMotorEx.class, "verticalSlide");
       // verticalMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void PositionAutoEx(){
        verticalMotor.setTargetPosition(POS_AUTO_EX_HEIGHT);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(1);
    }

    public boolean IsPositionAutoEx(){
        return verticalMotor.getCurrentPosition() >= (POS_AUTO_EX_HEIGHT - 20);
    }

    public void PositionAEx(){
        verticalMotor.setTargetPosition(POS_A_HEIGHT);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(1);
    }

    public boolean IsPositionAEx(){
        return verticalMotor.getCurrentPosition() >= (POS_A_HEIGHT - 20);
    }

    public void AutoPosition(){

        verticalMotor.setTargetPosition(AUTO_HEIGHT);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(1);
    }

    public boolean IsAutoPosition(){

        return verticalMotor.getCurrentPosition() >= (AUTO_HEIGHT - 20);
    }


    public boolean HasMoveFrom0to1Completed(){

        if(verticalMotor.getCurrentPosition() > POS_0_HEIGHT && (verticalMotor.getCurrentPosition() < (POS_0_HEIGHT + 100)))
        {
            return true;
        }
        return false;
    }
    public boolean HasMoveFrom1to2Completed(){

        if(verticalMotor.getCurrentPosition() > POS_1_HEIGHT && (verticalMotor.getCurrentPosition() < (POS_1_HEIGHT + 100)))
        {
            return true;
        }
        return false;
    }

    public void Position2(){
        verticalMotor.setTargetPosition(POS_2_HEIGHT);
        verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor.setPower(1);
    }

    public boolean IsPosition0(){
        return verticalMotor.getCurrentPosition() >= (POS_0_HEIGHT - 20);
    }

    public void Position0(){
        verticalMotor.setTargetPosition(POS_0_HEIGHT);
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

