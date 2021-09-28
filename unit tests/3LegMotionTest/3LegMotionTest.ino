/* Tests for:
 *  Kinematics accuracy
 *  Plug-in is necessary.
 */
#define DEBUG

#include <Leg.h>
#include <Timer.h>
RServoDriver servo_driver;
DogLeg leg;

enum Direction {X, Y, Z};

#define WISHBONE_ANGLE 155
#define DEFAULT_LEG_HEIGHT 100 //mm
#define WIDTH2 60 // half total width
#define LENGTH2 112.5 // half total length

#define LEG_UR_C_ANG_OFS  (0)
#define LEG_UR_S_ANG_OFS  (3)
#define LEG_UR_E_ANG_OFS  (-6)

#define LEG_BR_C_ANG_OFS  (0)
#define LEG_BR_S_ANG_OFS  (-3.5)
#define LEG_BR_E_ANG_OFS  (-1)

#define LEG_BL_C_ANG_OFS  (0)
#define LEG_BL_S_ANG_OFS  (-1)
#define LEG_BL_E_ANG_OFS  (5)

#define LEG_UL_C_ANG_OFS  (0)
#define LEG_UL_S_ANG_OFS  (1)
#define LEG_UL_E_ANG_OFS  (8)

// Servo Calibration Tables
#define NUM_TABLE_ELEM 19
int table_chest_ur[NUM_TABLE_ELEM]    = {1320,1433,1545,1658,1770,1900,2030,2160,2290,2420,2520,2620,2720,2820,2920,3053,3185,3318,3450};
int table_shoulder_ur[NUM_TABLE_ELEM] = {1380,1480,1605,1710,1830,1960,2065,2195,2315,2440,2545,2655,2770,2875,2985,3110,3210,3330,3440};
int table_elbow_ur[NUM_TABLE_ELEM]    = {1410,1530,1640,1760,1870,1980,2110,2220,2330,2440,2550,2660,2770,2880,2990,3080,3240,3349,3459};
int table_chest_br[NUM_TABLE_ELEM]    = {1430,1548,1665,1783,1900,2004,2108,2212,2316,2420,2536,2652,2768,2884,3000,3122,3245,3367,3490};
int table_shoulder_br[NUM_TABLE_ELEM] = {1380,1500,1620,1720,1850,1980,2105,2210,2325,2440,2540,2665,2800,2900,3030,3150,3255,3360,3475};
int table_elbow_br[NUM_TABLE_ELEM]    = {1420,1505,1600,1720,1830,1950,2060,2200,2320,2440,2550,2660,2775,2890,3000,3120,3240,3349,3459};
int table_chest_bl[NUM_TABLE_ELEM]    = {1370,1490,1610,1730,1850,1958,2066,2174,2282,2390,2498,2606,2714,2822,2930,3042,3155,3267,3380};
int table_shoulder_bl[NUM_TABLE_ELEM] = {1340,1450,1560,1690,1805,1930,2060,2180,2320,2440,2555,2670,2785,2895,3000,3110,3230,3340,3455};
int table_elbow_bl[NUM_TABLE_ELEM]    = {1387,1498,1614,1750,1855,1980,2110,2210,2330,2440,2550,2690,2795,2920,3020,3150,3260,3380,3480};
int table_chest_ul[NUM_TABLE_ELEM]    = {1350,1460,1570,1680,1790,1908,2026,2144,2262,2380,2490,2600,2710,2820,2930,3055,3180,3305,3430};
int table_shoulder_ul[NUM_TABLE_ELEM] = {1370,1480,1600,1715,1840,1950,2080,2190,2320,2440,2560,2680,2790,2900,3010,3140,3245,3345,3460};
int table_elbow_ul[NUM_TABLE_ELEM]    = {1387,1498,1614,1700,1810,1940,2060,2175,2315,2440,2560,2680,2780,2915,3020,3125,3240,3340,3445};

Timer timer;
Timer leg_update_timer;
#define LEG_UPDATE_PERIOD (1.0/1000) // s
#define NUM_LEGS 4
DogLeg leg_ur;
DogLeg leg_ul;
DogLeg leg_br;
DogLeg leg_bl;

DogLeg *foot[NUM_LEGS]; 

void setup() {
//  // put your setup code here, to run once:
    Serial.begin(9600);
    while (!Serial) {}
    initialize();
        // Reset
    reset();

    foot[3]->moveToPositionFromBody(Point(70, 60, -100)); // yaw -30: 67, -109
    foot[3]->operate();

    straightLineTest(X);
    
//Timer stopwatch;
//    // First, make sure angles look right
//    stopwatch.usePrecision();
//    for (int i = 0; i < NUM_LEGS; i++) {
//      stopwatch.reset();
//      foot[i]->gotoAngles(0, 45, -45 + WISHBONE_ANGLE);
//      Serial.println(stopwatch.dt()*1000);
//    }
//
//    delay(5000);
//
//    // Reset
//    for (int i = 0; i < NUM_LEGS; i++) {
//      foot[i]->moveToDefaultPosition();
//      foot[i]->operate();
//    }
//
//    // Then make sure IKIN is correct
//    for (int i = 0; i < NUM_LEGS; i++) {
//      stopwatch.reset();
//      foot[i]->moveToPositionFromBody(foot[i]->getDefaultPosition_oBfB()/Rot(0,0,30), 3);
//      Serial.println(stopwatch.dt()*1000);
//    }
//
//    timer.reset(3);
//    while(!timer.timeOut()) {
//      for (int i = 0; i < NUM_LEGS; i++) {
//        stopwatch.reset();
//            foot[i]->operate();
//            Serial.println(stopwatch.dt()*1000);
//      }
//    }

}

void loop() {
}

void straightLineTest(Direction direction) {
    Point foot_goal;
    if (direction == X)
      foot_goal = Point(20, 0, 0);
    else if (direction == Y) 
      foot_goal = Point(0, 20, 0);
    else {
      foot_goal = Point(0, 0, 20);
    }
    reset();
    float time = 1;
    int state = 1;
    Timer state_timer(time+0.1);
    while(1) {
        if (state == 0) {
            if (state_timer.timeOut()) {
                state_timer.reset();
                for (int i = 0; i < NUM_LEGS; i++) {
                    foot[i]->moveToPositionFromBodyInTime(foot[i]->getDefaultPosition_oBfB() + foot_goal, time);
                }
                state = 1;
            }
        } else {
            if (state_timer.timeOut()) {
                state_timer.reset();
                for (int i = 0; i < NUM_LEGS; i++) {
                    foot[i]->moveToPositionFromBodyInTime(foot[i]->getDefaultPosition_oBfB() - foot_goal, time);
                }
                state = 0;
            }
        }
        
        //if (leg_update_timer.timeOut()) {
            leg_update_timer.reset();
            for (int i = 0; i < NUM_LEGS; i++) {
                foot[i]->solveMotion();
            }
            for (int i = 0; i < NUM_LEGS; i++) {
                if (foot[i]->kinematicsIsValid())
                    foot[i]->sendSignalsAndSavePosition();
            }
        //}

    }
}

void reset() {

}


void initialize() {
      servo_driver.defaultStartup();

    // Set up legs
    foot[0] = &leg_ur;
    foot[1] = &leg_br;
    foot[2] = &leg_bl;
    foot[3] = &leg_ul;
    
    Point mounting_point;
    Point starting_position = Point(0, 0, DEFAULT_LEG_HEIGHT);

    mounting_point = Point( LENGTH2, -WIDTH2, 0);
    leg_ur = DogLeg(&servo_driver,  0,  1,  2, mounting_point, mounting_point - starting_position); // in future to enforce level starting, find way to set default_position.z = default_height conveniently.
    leg_ur.flipLR();
    leg_ur.setSignalTables(table_chest_ur, table_shoulder_ur, table_elbow_ur);
    leg_ur.calibrateServos(LEG_UR_C_ANG_OFS, LEG_UR_S_ANG_OFS, LEG_UR_E_ANG_OFS);
    leg_ur.setID(0);

    mounting_point = Point(-LENGTH2, -WIDTH2, 0);
    leg_br = DogLeg(&servo_driver,  4,  5,  6, mounting_point, mounting_point - starting_position);
    leg_br.flipLR();
    leg_br.flipFB();
    leg_br.setSignalTables(table_chest_br, table_shoulder_br, table_elbow_br);
    leg_br.calibrateServos(LEG_BR_C_ANG_OFS, LEG_BR_S_ANG_OFS, LEG_BR_E_ANG_OFS);
    leg_br.setID(1);

    mounting_point = Point(-LENGTH2, WIDTH2, 0);
    leg_bl = DogLeg(&servo_driver,  8,  9, 10, mounting_point, mounting_point - starting_position);
    leg_bl.flipFB();
    leg_bl.setSignalTables(table_chest_bl, table_shoulder_bl, table_elbow_bl);
    leg_bl.calibrateServos(LEG_BL_C_ANG_OFS, LEG_BL_S_ANG_OFS, LEG_BL_E_ANG_OFS);
    leg_bl.setID(2);

    mounting_point = Point( LENGTH2, WIDTH2, 0);
    leg_ul = DogLeg(&servo_driver, 12, 13, 14, mounting_point, mounting_point - starting_position);
    leg_ul.setSignalTables(table_chest_ul, table_shoulder_ul, table_elbow_ul);
    leg_ul.calibrateServos(LEG_UL_C_ANG_OFS, LEG_UL_S_ANG_OFS, LEG_UL_E_ANG_OFS);
    leg_ul.setID(3);

    reset();

    leg_update_timer.usePrecision();
    leg_update_timer.reset(LEG_UPDATE_PERIOD);
}
