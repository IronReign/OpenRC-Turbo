package org.firstinspires.ftc.teamcode.path;

import android.location.Location;

/*
Defines a path of gps coordinates that spell out 6832, starting from the tail of the 2
situated at Northhaven park's softball field
// points2 doubles back at the end all the way to index [2]
    //points3a doubles back at the end all the way to index [2]
    //points3b doubles back to index [0]
    // points68 does terminates at the last point
    //see the original kml files in ftc2020/beachcomber in google earth
 */
public class teamnumberpath {

    double[] points2 = {-96.84830341261102,32.90264785232041,-96.84831936220702,32.90265891740987,-96.84834341640983,32.90267411910521,-96.84832990550969,32.90267568052462,-96.84831805204043,32.90267712329844,-96.84830382927584,32.90267942348304,-96.84828674335144,32.9026828871812,-96.84827623039766,32.90268601340104,-96.84826659814816,32.90269314092458,-96.84826302416829,32.90270304824498,-96.8482639272727,32.90270907372624,-96.84826617124526,32.90271514738101,-96.84827397681501,32.90272020465739,-96.84828274965172,32.90272247946277,-96.84829034667098,32.90272197552419,-96.84829930101779,32.90271885201253,-96.84830526786504,32.90271326831061};
    double[] points3a = {-96.84834933871083,32.90267834576805,-96.84836344095297,32.90268705142343,-96.84837602591732,32.90269539595939,-96.84838493183091,32.90270520192659,-96.84838495337743,32.9027139320755,-96.8483808594729,32.90272285126611};
    double[] points3b = {-96.84836419619812,32.90269159708182,-96.84835336578691,32.90269197967941,-96.84834557801705,32.9026963583192,-96.84833974392039,32.90270237047054,-96.84833737853108,32.90270946786195,-96.84833869758714,32.90271728635938,-96.84834152763879,32.90272291968464,-96.84834955986443,32.9027298171124,-96.84833915058435,32.90272528824412,-96.84833005048468,32.90272457655716,-96.84832247439677,32.90272695416235,-96.84831685467462,32.90273205654765,-96.84831448883502,32.90273897206084,-96.84831602316638,32.9027462445518,-96.84831972017379,32.90275205824972,-96.84832861217559,32.90275622599906,-96.84834009247744,32.90275584227413,-96.84835004652071,32.90275145986261};
    double[] points68 = {-96.84838162058642,32.90269946710981,-96.84839376491418,32.90270814382628,-96.84841054482496,32.90271958837995,-96.84842488423217,32.9027291279214,-96.84843071176448,32.90273469631649,-96.84843421574406,32.90273801277197,-96.84843562635334,32.90274264401332,-96.84843454539383,32.90274963215212,-96.84843053102625,32.90275760847532,-96.84842330602704,32.90276270150024,-96.84841352169144,32.90276473796586,-96.84840337982249,32.90276364524875,-96.84839499625598,32.90276051380724,-96.8483875801313,32.90275758359248,-96.84838010480379,32.90275739736447,-96.84837263439267,32.90275920346166,-96.84836754358288,32.90276369507007,-96.8483643540917,32.90276937877162,-96.84836353215779,32.90277286678607,-96.84836395644166,32.90277629795661,-96.84836623138639,32.90278189089537,-96.84837124419676,32.90278628499882,-96.84837940205239,32.90279013174376,-96.84838831324434,32.902790333534,-96.84839595276867,32.9027871682847,-96.84840077959693,32.90278178570697,-96.84840296731132,32.90277511972182,-96.84840148975468,32.90276881608001,-96.84839732157701,32.90276254747927,-96.84839079602352,32.90275552448042,-96.84838724790416,32.90274683280874,-96.84838876934717,32.90273813231755,-96.84839272130074,32.90273183338462,-96.84839976455457,32.90272774979918,-96.84840923557746,32.90272495741256,-96.84841937698411,32.9027258650687,-96.8484304146745,32.90273269310066,-96.84844057207467,32.90274007784572,-96.84845205425657,32.90274838558814,-96.8484644153679,32.90275558143805,-96.84847391016071,32.90276241215427,-96.84848010100403,32.90277017392577,-96.84848210759688,32.9027794234656,-96.84847705996927,32.90278831515884,-96.8484662783189,32.90279592141015,-96.84845636233612,32.90279704903175,-96.84844467206467,32.90279373827834,-96.84843870347073,32.90278671636175,-96.84843735492912,32.90277617025149,-96.84844196355965,32.902768019566,-96.84845076511834,32.90276189725228,-96.84846111644227,32.90275854814828,-96.84847059843902,32.90276019719064,-96.84847854758014,32.90276592023645,-96.84848187529948,32.90277461229307,-96.84848013711068,32.90278479365254,-96.84847156236494,32.90279350642481,-96.84846077340175,32.90279815171763,-96.8484477784523,32.90280206060567,-96.84843544429158,32.90280578328134,-96.84842575329604,32.90280876111481,-96.84840901401408,32.90281378687825};

    private int index = -1;
    private boolean forward = true;
    private boolean done = false;

    double[] points = {-96.84060471247678,32.86930628121518,-96.84061961847621,32.86924701358645,-96.84054823419424,32.8692343320722,-96.84053344789302,32.869295362769,-96.84060471247678,32.86930628121518};
    int character = 0;
    Location loc = new Location("");

    public int getChar(){return character;}
    public void setChar(int character){this.character=character;}
    public int getindex(){return index;}

    public Location getNext(){
        switch (character) {
            case 0: //2
                if ((index+1) * 2 == points2.length) forward = false;
                if (forward) {
                    index++;
                    loc.setLongitude(points2[index*2]);
                    loc.setLatitude(points2[index*2+1]);
                }
                else { //doubling back to index 2
                    index--;
                    loc.setLongitude(points2[index*2]);
                    loc.setLatitude(points2[index*2+1]);
                    if (index == 2) {
                        index = -1;
                        character++;
                        forward = true;
                        return loc;
                    }
                }
                return loc;
            case 1: //3a
                if ((index+1) * 2 == points3a.length) forward = false;
                if (forward) {
                    index++;
                    loc.setLongitude(points3a[index*2]);
                    loc.setLatitude(points3a[index*2+1]);
                }
                else { //doubling back to index 2
                    index--;
                    loc.setLongitude(points3a[index*2]);
                    loc.setLatitude(points3a[index*2+1]);
                    if (index == 1) {
                        index = -1;
                        character++;
                        forward = true;
                        return loc;
                    }
                }
                return loc;
            case 2: //3b
                if ((index+1) * 2 == points3b.length) forward = false;
                if (forward) {
                    index++;
                    loc.setLongitude(points3b[index*2]);
                    loc.setLatitude(points3b[index*2+1]);
                }
                else { //doubling back all the way to index 0
                    index--;
                    loc.setLongitude(points3b[index*2]);
                    loc.setLatitude(points3b[index*2+1]);
                    if (index == 0) {
                        index = -1;
                        character++;
                        forward = true;
                        return loc;
                    }
                }
                return loc;
            case 3: //68
                index++;
                loc.setLongitude(points68[index*2]);
                loc.setLatitude(points68[index*2+1]);
                if ((index+1) * 2 == points68.length){
                    index = -1;
                    character = 0;
                    done = true;
                }
                return loc;
        }
        return loc;
    }

    public boolean isDone() {
        return done;
    }

}


