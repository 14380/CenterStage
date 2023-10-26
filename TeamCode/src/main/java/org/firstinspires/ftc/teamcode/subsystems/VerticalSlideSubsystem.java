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

    public VerticalSlideSubsystem(HardwareMap map, RobotStateSubsystem state){
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

    public void Position1(){
        //TODO: Work out the Position 1 pos
    }

    public boolean IsPosition1(){
        //TODO: check to ensure that the encoders are past Pos 1 and lower than Pos 2
        return true;
    }

    public void Position2(){
        //TODO: Move to Pos 2 (this all the way up?)
    }

    public boolean IsPosition2(){
        //TODO: check to ensure that we are greater than Pos 2
        return true;
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

