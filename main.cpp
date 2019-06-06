#include "mbed-os/mbed.h"
#include "mbed_error.h"
#include "MBed_Adafruit-GPS-Library/MBed_Adafruit_GPS.h"
#include "X_NUCLEO_IKS01A2/XNucleoIKS01A2.h"
#include "HTS221/HTS221Sensor.h"
#include "LPS22HB/LPS22HBSensor.h"
#include "LSM303AGR/LSM303AGRAccSensor.h"
#include "LSM303AGR/LSM303AGRMagSensor.h"
#include "LSM6DSL/LSM6DSLSensor.h"


#define MBED_CONF_PLATFORM_ERROR_FILENAME_CAPTURE_ENABLED 0

//Configures pins and serial port
Serial pc(USBTX, USBRX);
InterruptIn PPS(A1, PullNone);
DigitalOut led(LED1);

//Configure GPS
Serial * gps_Serial = new Serial(D1,D0); //serial object for use w/ GPS
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
char c; //when read via Adafruit_GPS::read(), the class returns single character stored here

/* Defines the two queues used, one for events and one for printing to the screen */
EventQueue printfQueue;
EventQueue eventQueue;
EventQueue GPSQueue;

/* Defines the timer */
Timer t;
time_t whattime;

int int_time=0, int_count=0;
int64_t usTime1 = 0, usTime2 = 0, usDeltaTime = 0;
bool UpdateTime = false;

/* Sensor Board Variables */
uint8_t id;
float temp1, temp2, humid1, humid2;
char buffer1[32], buffer2[32], buffer3[32], buffer4[32];
int32_t axes1[3], axes2[3], axes3[3], axes4[3];

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM303AGRMagSensor *magnetometer = mems_expansion_board->magnetometer;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;
static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
static LSM303AGRAccSensor *accelerometer = mems_expansion_board->accelerometer;

/* Helper function for printing floats & doubles */
static char *print_double(char* str, double v, int decimalDigits=2)
{
    int i = 1;
    int intPart, fractPart;
    int len;
    char *ptr;

    /* prepare decimal digits multiplicator */
    for (;decimalDigits!=0; i*=10, decimalDigits--);

    /* calculate integer & fractinal parts */
    intPart = (int)v;
    fractPart = (int)((v-(double)(int)v)*i);

    /* fill in integer part */
    sprintf(str, "%i.", intPart);

    /* prepare fill in of fractional part */
    len = strlen(str);
    ptr = &str[len];

    /* fill in leading fractional zeros */
    for (i/=10;i>1; i/=10, ptr++) {
        if (fractPart >= i) {
        break;
        }
        *ptr = '0';
    }

    /* fill in (rest of) fractional part */
    sprintf(ptr, "%i", fractPart);

    return str;
}

void Detect1PPS() {
    led = !led;
}

/* Converts standard time into Epoch time. Could delete this if no longer needed.*/
time_t asUnixTime(int year, int mon, int mday, int hour, int min, int sec) {
    struct tm   t;
    t.tm_year = year - 1900;
    t.tm_mon =  mon - 1;        // convert to 0 based month
    t.tm_mday = mday;
    t.tm_hour = hour;
    t.tm_min = min;
    t.tm_sec = sec;
    t.tm_isdst = -1;            // Is Daylight saving time on? 1 = yes, 0 = no, -1 = unknown
 
    return mktime(&t);          // returns seconds elapsed since January 1, 1970 (begin of the Epoch)
}

/* Prints to the serial console */
void Print_Sensors() {
    // this runs in the lower priority thread
    pc.printf("%d ", whattime);
    pc.printf("%d ", int_time);
    pc.printf("%lld ", usDeltaTime);
    pc.printf("%7s %s ", print_double(buffer1, temp1), print_double(buffer2, humid1));
    pc.printf("%7s %s ", print_double(buffer3, temp2), print_double(buffer4, humid2));
    pc.printf("%6ld %6ld %6ld ", axes1[0], axes1[1], axes1[2]);
    pc.printf("%6ld %6ld %6ld", axes2[0], axes2[1], axes2[2]);
    pc.printf("%6ld %6ld %6ld", axes3[0], axes3[1], axes3[2]);
    pc.printf("%6ld %6ld %6ld\r\n", axes4[0], axes4[1], axes4[2]);
}

/* Reads the sensor data */
void Read_Sensors() {
    hum_temp->get_temperature(&temp1);
    hum_temp->get_humidity(&humid1);
    press_temp->get_temperature(&temp2);
    press_temp->get_pressure(&humid2);
    magnetometer->get_m_axes(axes1);
    accelerometer->get_x_axes(axes2);
    acc_gyro->get_x_axes(axes3);
    acc_gyro->get_g_axes(axes4);
    Detect1PPS();
    Print_Sensors();
}

//Collects and parses GPS data
//This runs in the high priority thread
void GPS_data() {
    do{
        c = myGPS.read();   //queries the GPS
        //if (c) { pc.printf("%c", c); } //this line will echo the GPS data if not paused
        //check if we recieved a new message from GPS, if so, attempt to parse it,
        if (myGPS.newNMEAreceived() == true)
        {
            if (myGPS.parse(myGPS.lastNMEA()) == true)
            {
                int_time = asUnixTime(myGPS.year+2000, myGPS.month, myGPS.day, myGPS.hour, myGPS.minute, myGPS.seconds);  
                if (UpdateTime == true)
                {
                    set_time(int_time+1);  //temp workaround for initial offset between GPS and micro
                }
                                
                break;
            }
        }
    } while(myGPS.newNMEAreceived() == false);
    whattime = time(NULL);
    usTime2 = t.read_high_resolution_us();
    usDeltaTime = usTime2 - usTime1;
    t.stop();
    t.reset();
    t.start();
    usTime1 = t.read_high_resolution_us();
    UpdateTime = false;
}

// main() runs in its own thread in the OS
int main()
{  
    myGPS.begin(57600);
    //myGPS.begin(9600);
    
    myGPS.sendCommand(PMTK_STANDBY);
    pc.printf("Entering GPS Standby...\r\n");
    wait(1);
    myGPS.sendCommand(PMTK_AWAKE);
    pc.printf("Wake Up GPS...\r\n");
    wait(1);
    myGPS.sendCommand(PMTK_SET_BAUD_57600);
    pc.printf("Set GPS Baud Rate to 57600...\r\n");
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    pc.printf("Set RM Message Format Only...\r\n");
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
    pc.printf("Set 10Hz Message Update Rate...\r\n");
    myGPS.sendCommand(PGCMD_NOANTENNA);
    pc.printf("Turn Off Antenna Messages...\r\n");

    /* Enable all sensors */
    hum_temp->enable();
    press_temp->enable();
    magnetometer->enable();
    accelerometer->enable();
    acc_gyro->enable_x();
    acc_gyro->enable_g();
    wait(1);

    pc.printf("Sensors Enabled...\r\n");

    /* resets and starts the timer */
    t.reset();
    t.start();
    usTime1 = t.read_high_resolution_us();

    pc.printf("Timer Reset and Started...\r\n");

    do{
        UpdateTime = true;
        GPS_data();   //queries the GPS
        pc.printf("Waiting for GPS Fix...%d\r\n",myGPS.fix);
        pc.printf("GPS Time...%d\r\n",int_time);
        pc.printf("Micro Time...%d\r\n",whattime);
        pc.printf("Counter Number...%d\r\n\r\n", int_count);
        int_count++;

    }while((myGPS.fix == false) || (int_count < 4));
    UpdateTime = false;

    //Prints headers for each measurement. Unsure if the acc, mag, and gyro
    //directions are accurate. (Don't know if accx actually measures in x direction)
    pc.printf("\r\nEPOC GPST Delta TEP1 HUM TEP2 PRES MAGX MAGY MAGZ AC1X AC1Y AC1Z AC2X AC2Y AC2Z GYRX GYRY GYRZ\r\n");
    
    // normal priority thread for other events
    Thread eventThread(osPriorityHigh);
    eventThread.start(callback(&eventQueue, &EventQueue::dispatch_forever));
  
    // low priority thread for calling printf()
    Thread printfThread(osPriorityLow);
    printfThread.start(callback(&printfQueue, &EventQueue::dispatch_forever));

    PPS.rise(eventQueue.event(&GPS_data));
    PPS.fall(printfQueue.event(&Read_Sensors));
    
    wait(osWaitForever);
}
