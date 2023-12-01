package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HorizontalSlideSubsystem extends SubsystemBase {

    private DcMotorEx horizontalMotor;

    private int EXTENDED_POS = 1950;
    private int RETRACT_POS = 35;

    public HorizontalSlideSubsystem(HardwareMap map, RobotStateSubsystem state){
        horizontalMotor = map.get(DcMotorEx.class, "horizontalSlide");
        //horizontalMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalMotor.setPower(0);


    }

    public void Extend(){
        horizontalMotor.setTargetPosition(EXTENDED_POS + 20);
        horizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalMotor.setPower(1);
    }

    public void Retract(){
        horizontalMotor.setTargetPosition(0);
        horizontalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalMotor.setPower(1);
    }

    public void ManualExtend(double speed){

        horizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(horizontalMotor.getCurrentPosition() < EXTENDED_POS){
            horizontalMotor.setPower(speed);

        }else{
            horizontalMotor.setPower(0);

        }

    }

    public void ManualZeroPower(){
        //this should brake the motor
        horizontalMotor.setPower(0);
    }

    public void ManualExtendOff(){
        horizontalMotor.setPower(0);
    }

    public boolean IsExtended(){
        return horizontalMotor.getCurrentPosition() >= (EXTENDED_POS - 150);
    }

    public int getCurrentPosition(){
        return horizontalMotor.getCurrentPosition();
    }

    public boolean IsRetracted(){
        return horizontalMotor.getCurrentPosition() <= RETRACT_POS;
    }
}
