package org.usfirst.ftc.aperturescience;

/**
 * AutoRedPosition1 (Autonomous)
 *
 * Slow and accurate.
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous(name="Auto Nothing", group="Neutral")
public class AutoNothing extends AutoCommon {

    // how far to stay away from the wall

    @Override
    public void main() throws InterruptedException {

        // everything up to waitForStart
        super.mainNoGyro();

        // pull out the tapes to make room for the arm
        setTapes();


    }
}