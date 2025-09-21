package org.firstinspires.ftc.teamcode.utilities;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Utility class for controlling the robot's intake mechanism.
 * This class encapsulates the logic for initializing and operating the intake motor.
 */
public class IntakeUtil {
    private final DcMotor intakeMotor;
    private final CRServo intakeServo;

    /** The default hardware map name for the intake motor. */
    public static final String INTAKE_MOTOR_NAME = "intake";
    /**
     * Constructs an IntakeUtil object.
     *
     * @param hardwareMap The HardwareMap from the OpMode, used to retrieve the intake motor.
     * @throws IllegalArgumentException if hardwareMap is null.
     */
    public IntakeUtil(HardwareMap hardwareMap) { // Pass HardwareMap in constructor
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null");
        }
        //Intake Motor (new bot)
        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
    }


    /**
     * Stops the intake motor by setting its power to zero.
     */
    public void stopIntake() {
        setIntakeMotorPower(0.0);
    }//Stop

    /**
     * Sets the power of the intake motor.
     *
     * @param /power The desired power level, typically between -1.0 and 1.0.
     */
    public void setIntakeMotorPower(double power) {
        intakeMotor.setPower(power);
        intakeServo.setPower(-power);
    }

    /**
     * Gets the current power of the intake motor.
     *
     * @return The current power level of the motor.
     */
    public double getIntakeMotorPower() {
        return intakeMotor.getPower();
    }

    /**
     * Gets the current run mode of the intake motor.
     *
     * @return The current DcMotor.RunMode.
     */
    public DcMotor.RunMode getIntakeMode(){
        return intakeMotor.getMode();
    }

}   //end program