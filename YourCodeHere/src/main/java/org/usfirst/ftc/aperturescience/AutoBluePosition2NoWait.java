package org.usfirst.ftc.aperturescience;

/**
 * AutoRedPosition2 (Autonomous)
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous(name="BLUE Beach Bum Jimmy NoWait", group="Blue")
public class AutoBluePosition2NoWait extends AutoCommon {

    // how far to stay away from the wall
    private final double THE_DISTANCE = 21;

    @Override
    public void main() throws InterruptedException {

        // everything up to waitForStart
        super.main();

        // pull out the tapes to make room for the arm
        setTapes();

        // pause for 6 seconds
        //Thread.sleep(6000); FIXME

        // move the robot toward the mountain
        driveBack(.2, 6);
        Thread.sleep(500);
        turnGyroSlow(95);
        Thread.sleep(500);
        driveBack(.2, 35);
        Thread.sleep(500);
        turnGyroSlow(-59);
        Thread.sleep(500);

        // drive back 90 inches or until we find the white line
        boolean foundWhite = driveBackToWhite(.6, 90);
        Thread.sleep(200);  // small pause

        if (foundWhite) {
            // yeah! found the white line

            // pause to let the gyro settle
            Thread.sleep(500);

            // turn left 49 degrees
            turnGyro(64);
            Thread.sleep(200);

            // drive back to wall using the ultrasonic
            driveToDistance(.2, 6, THE_DISTANCE);

        } else {
            // did not find white line -- let's try to correct

            // drive forward 10 inches
            drive(.3, 14);
            Thread.sleep(500);

            // turn left 56 degrees
            turnGyro(70);
            Thread.sleep(500);

            // drive back to wall using ultrasonic
            driveToDistance(.2, 6, THE_DISTANCE);
        }

        // bring the arm up
        autoArmUp();

        dropBucket();
        Thread.sleep(1000);

        // bring the arm back into the robot to get ready for TeleOp
        autoArmDown();
    }
}