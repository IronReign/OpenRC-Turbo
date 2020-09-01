package org.firstinspires.ftc.teamcode.path;

import android.location.Location;

/* this is a simple square endless path on the front lawn of 9512 folkstone 75220 */

public class bluesquarepath {

    private int index = -1;
    double[] points = {-96.84060471247678,32.86930628121518,-96.84061961847621,32.86924701358645,-96.84054823419424,32.8692343320722,-96.84053344789302,32.869295362769,-96.84060471247678,32.86930628121518};

public Location getNext(){
    if (index == 7) index = -1;
    Location loc = new Location("");
    loc.setLongitude(points[++index]);
    loc.setLatitude(points[++index]);
    return loc;
}

public boolean isDone() {
    return false;
} //this path is endless

}
