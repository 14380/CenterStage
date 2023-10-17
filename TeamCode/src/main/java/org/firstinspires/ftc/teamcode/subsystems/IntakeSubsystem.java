package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {

    private DcMotorEx intakeMotor;
    private CRServo intakeServo;

    private DistanceSensor sensorDistanceTop;
    private DistanceSensor sensorDistanceBottom;

    public IntakeSubsystem(HardwareMap map){
        intakeMotor = map.get(DcMotorEx.class, "intake");
        intakeServo = map.get(CRServo.class, "intakeServo");
        sensorDistanceTop = map.get(DistanceSensor.class, "topC");
        sensorDistanceBottom = map.get(DistanceSensor.class, "bottomC");

        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public boolean IsTopCovered(){
        double distance = sensorDistanceTop.getDistance(DistanceUnit.MM);

        if(distance <= 50){
            return true;
        }
        return false;
    }

    public boolean IsBottomCovered(){
        double distance = sensorDistanceBottom.getDistance(DistanceUnit.MM);

        if(distance <= 15){
            return true;
        }
        return false;
    }

    public double getTopDistance(){
        return sensorDistanceTop.getDistance(DistanceUnit.MM);
    }

    public double getBottomDistance(){
        return sensorDistanceBottom.getDistance(DistanceUnit.MM);
    }
    public void IntakeOn(){
        intakeMotor.setPower(1);
        if(!IsTopCovered()){
            IntakeAdvanceSpeed(0.2);
        }else{
            //turn off the advancement if the top is covered.
            IntakeAdvanceSpeed(0);
        }

    }

    public void IntakeAdvanceSpeed(double speed){
        intakeServo.setPower(speed);
    }

    public void IntakeAdvance(){
        intakeServo.setPower(1);
    }

    public void IntakeOff(){
        intakeMotor.setPower(0);
        intakeServo.setPower(0);
    }

    public void Reverse(){
        intakeMotor.setPower(-1);
        intakeServo.setPower(-1);
    }
}
