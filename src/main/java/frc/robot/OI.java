package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
    private static Joystick left = new Joystick(0);
    private static Joystick right = new Joystick(1);

    public static double getLeftX() {
        return left.getX()*0.7;
    }

    public static double getLeftY() {
        return -left.getY()*0.7;
    }

    public static double getRightX() {

        return right.getX()*0.7;
    }

    public static double getRightY() {
        return -right.getY()*0.7;
    }
}
