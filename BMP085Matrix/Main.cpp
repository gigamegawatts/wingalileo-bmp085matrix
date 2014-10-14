// Main.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "arduino.h"
#include "DateTime.h"
#include "GMBmp085.h"
#include "GMCosm.h"
#include "GMMaxMatrix.h"
#include "fstream"
#include "iostream"
using namespace std;

int readCPUTemp();
void printString(char* s);
int readIni(char * filename);
string delSpaces(string &str);

//---------------- USER CONFIGURABLE SETTINGS --------------------
// ------------------- Cosm (Xively) settings ---------------
// NOTE - these can also be put in the BMP085Matrix.ini file on your Galileo
#define COSM_API_KEY ""
#define COSM_FEED ""
#define COSM_CPU_TEMP_DATASTREAM ""
#define COSM_TEMP_DATASTREAM ""
#define COSM_PRESSURE_DATASTREAM ""
// upload interval, in ms
#define COSM_UPLOAD_INTERVAL 300*1000 
// ------------------ measurement settings -------------------
// interval at which to take new measurements, in ms
#define MEASUREMENT_INTERVAL 30*1000
// --------------- LED matrix settings ---------------------
// number of 8x8 matrixes
#define NUM_MATRIX 3
// pin connected to matrix CS
#define MATRIX_CS_PIN 2
// max characters to be written to matrix (including ones that scroll by)
#define MATRIX_DATA_LEN 50

string cosmAPIKey = COSM_API_KEY;
string cosmFeed = COSM_FEED;
string cosmCPUTempDatastream = COSM_CPU_TEMP_DATASTREAM;
string cosmTempDatastream = COSM_TEMP_DATASTREAM;
string cosmPressureDatastream = COSM_PRESSURE_DATASTREAM;

long total_cpu_temp = 0;
double total_bmp085_temp = 0;
double total_bmp085_pressure = 0;
int total_readings = 0;
// this is initialized to force an initial reading the first time we loop through the code
unsigned long last_reading_millis = 1000000;
unsigned long last_upload_millis = 0;
// most recent temperature (C) returned from BMP085
float bmptemp;
// most recent pressure (Pascals) returned from BMP085
int32_t bmppress;
// most recent pressure (kPa) returned from BMP085
double bmppressK;

GMBmp085 bmp;
GMCosm cosm;

// bitmap for LED matrix
byte CH[] = {
	1, 8, B0000000, B0000000, B0000000, B0000000, B0000000, // space
	1, 8, B1011111, B0000000, B0000000, B0000000, B0000000, // !
	3, 8, B0000011, B0000000, B0000011, B0000000, B0000000, // "
	5, 8, B0010100, B0111110, B0010100, B0111110, B0010100, // #
	4, 8, B0100100, B1101010, B0101011, B0010010, B0000000, // $
	5, 8, B1100011, B0010011, B0001000, B1100100, B1100011, // %
	5, 8, B0110110, B1001001, B1010110, B0100000, B1010000, // &
	1, 8, B0000011, B0000000, B0000000, B0000000, B0000000, // '
	3, 8, B0011100, B0100010, B1000001, B0000000, B0000000, // (
	3, 8, B1000001, B0100010, B0011100, B0000000, B0000000, // )
	5, 8, B0101000, B0011000, B0001110, B0011000, B0101000, // *
	5, 8, B0001000, B0001000, B0111110, B0001000, B0001000, // +
	2, 8, B10110000, B1110000, B0000000, B0000000, B0000000, // ,
	4, 8, B0001000, B0001000, B0001000, B0001000, B0000000, // -
	2, 8, B1100000, B1100000, B0000000, B0000000, B0000000, // .
	4, 8, B1100000, B0011000, B0000110, B0000001, B0000000, // /
	4, 8, B0111110, B1000001, B1000001, B0111110, B0000000, // 0
	3, 8, B1000010, B1111111, B1000000, B0000000, B0000000, // 1
	4, 8, B1100010, B1010001, B1001001, B1000110, B0000000, // 2
	4, 8, B0100010, B1000001, B1001001, B0110110, B0000000, // 3
	4, 8, B0011000, B0010100, B0010010, B1111111, B0000000, // 4
	4, 8, B0100111, B1000101, B1000101, B0111001, B0000000, // 5
	4, 8, B0111110, B1001001, B1001001, B0110000, B0000000, // 6
	4, 8, B1100001, B0010001, B0001001, B0000111, B0000000, // 7
	4, 8, B0110110, B1001001, B1001001, B0110110, B0000000, // 8
	4, 8, B0000110, B1001001, B1001001, B0111110, B0000000, // 9
	1, 8, B01010000, B0000000, B0000000, B0000000, B0000000, // :
	2, 8, B10000000, B01010000, B0000000, B0000000, B0000000, // ;
	3, 8, B0010000, B0101000, B1000100, B0000000, B0000000, // <
	3, 8, B0010100, B0010100, B0010100, B0000000, B0000000, // =
	3, 8, B1000100, B0101000, B0010000, B0000000, B0000000, // >
	4, 8, B0000010, B1011001, B0001001, B0000110, B0000000, // ?
	5, 8, B0111110, B1001001, B1010101, B1011101, B0001110, // @
	4, 8, B1111110, B0010001, B0010001, B1111110, B0000000, // A
	4, 8, B1111111, B1001001, B1001001, B0110110, B0000000, // B
	4, 8, B0111110, B1000001, B1000001, B0100010, B0000000, // C
	4, 8, B1111111, B1000001, B1000001, B0111110, B0000000, // D
	4, 8, B1111111, B1001001, B1001001, B1000001, B0000000, // E
	4, 8, B1111111, B0001001, B0001001, B0000001, B0000000, // F
	4, 8, B0111110, B1000001, B1001001, B1111010, B0000000, // G
	4, 8, B1111111, B0001000, B0001000, B1111111, B0000000, // H
	3, 8, B1000001, B1111111, B1000001, B0000000, B0000000, // I
	4, 8, B0110000, B1000000, B1000001, B0111111, B0000000, // J
	4, 8, B1111111, B0001000, B0010100, B1100011, B0000000, // K
	4, 8, B1111111, B1000000, B1000000, B1000000, B0000000, // L
	5, 8, B1111111, B0000010, B0001100, B0000010, B1111111, // M
	5, 8, B1111111, B0000100, B0001000, B0010000, B1111111, // N
	4, 8, B0111110, B1000001, B1000001, B0111110, B0000000, // O
	4, 8, B1111111, B0001001, B0001001, B0000110, B0000000, // P
	4, 8, B0111110, B1000001, B1000001, B10111110, B0000000, // Q
	4, 8, B1111111, B0001001, B0001001, B1110110, B0000000, // R
	4, 8, B1000110, B1001001, B1001001, B0110010, B0000000, // S
	5, 8, B0000001, B0000001, B1111111, B0000001, B0000001, // T
	4, 8, B0111111, B1000000, B1000000, B0111111, B0000000, // U
	5, 8, B0001111, B0110000, B1000000, B0110000, B0001111, // V
	5, 8, B0111111, B1000000, B0111000, B1000000, B0111111, // W
	5, 8, B1100011, B0010100, B0001000, B0010100, B1100011, // X
	5, 8, B0000111, B0001000, B1110000, B0001000, B0000111, // Y
	4, 8, B1100001, B1010001, B1001001, B1000111, B0000000, // Z
	2, 8, B1111111, B1000001, B0000000, B0000000, B0000000, // [
	4, 8, B0000001, B0000110, B0011000, B1100000, B0000000, // backslash
	2, 8, B1000001, B1111111, B0000000, B0000000, B0000000, // ]
	3, 8, B0000010, B0000001, B0000010, B0000000, B0000000, // hat
	4, 8, B1000000, B1000000, B1000000, B1000000, B0000000, // _
	2, 8, B0000001, B0000010, B0000000, B0000000, B0000000, // `
	4, 8, B0100000, B1010100, B1010100, B1111000, B0000000, // a
	4, 8, B1111111, B1000100, B1000100, B0111000, B0000000, // b
	4, 8, B0111000, B1000100, B1000100, B0101000, B0000000, // c
	4, 8, B0111000, B1000100, B1000100, B1111111, B0000000, // d
	4, 8, B0111000, B1010100, B1010100, B0011000, B0000000, // e
	3, 8, B0000100, B1111110, B0000101, B0000000, B0000000, // f
	4, 8, B10011000, B10100100, B10100100, B01111000, B0000000, // g
	4, 8, B1111111, B0000100, B0000100, B1111000, B0000000, // h
	3, 8, B1000100, B1111101, B1000000, B0000000, B0000000, // i
	4, 8, B1000000, B10000000, B10000100, B1111101, B0000000, // j
	4, 8, B1111111, B0010000, B0101000, B1000100, B0000000, // k
	3, 8, B1000001, B1111111, B1000000, B0000000, B0000000, // l
	5, 8, B1111100, B0000100, B1111100, B0000100, B1111000, // m
	4, 8, B1111100, B0000100, B0000100, B1111000, B0000000, // n
	4, 8, B0111000, B1000100, B1000100, B0111000, B0000000, // o
	4, 8, B11111100, B0100100, B0100100, B0011000, B0000000, // p
	4, 8, B0011000, B0100100, B0100100, B11111100, B0000000, // q
	4, 8, B1111100, B0001000, B0000100, B0000100, B0000000, // r
	4, 8, B1001000, B1010100, B1010100, B0100100, B0000000, // s
	3, 8, B0000100, B0111111, B1000100, B0000000, B0000000, // t
	4, 8, B0111100, B1000000, B1000000, B1111100, B0000000, // u
	5, 8, B0011100, B0100000, B1000000, B0100000, B0011100, // v
	5, 8, B0111100, B1000000, B0111100, B1000000, B0111100, // w
	5, 8, B1000100, B0101000, B0010000, B0101000, B1000100, // x
	4, 8, B10011100, B10100000, B10100000, B1111100, B0000000, // y
	3, 8, B1100100, B1010100, B1001100, B0000000, B0000000, // z
	3, 8, B0001000, B0110110, B1000001, B0000000, B0000000, // {
	1, 8, B1111111, B0000000, B0000000, B0000000, B0000000, // |
	3, 8, B1000001, B0110110, B0001000, B0000000, B0000000, // }
	4, 8, B0001000, B0000100, B0001000, B0000100, B0000000, // ~
};
int cs = MATRIX_CS_PIN; // SPI CS pin 
int matrixCount = NUM_MATRIX;    //change this variable to set how many MAX7219's you'll use

GMMaxMatrix m(cs, matrixCount);

// buffer used to copy 1 character's bitmap into LED matrix buffer - generally only requires 8 bytes
byte buffer[10];
// column index in LED matrix, including off-screen text that scrolls onto the buffer
int column = 0;

char matrixData[MATRIX_DATA_LEN];

int loop_count = 0;
DateTime dt;

int _tmain(int argc, _TCHAR* argv[])
{
	return RunArduinoSketch();
}



void setup()
{
	if (readIni("BMP085Matrix.ini") != 0)
	{
		Log("unable to read .ini file, bye\n");
		exit(-1);
	}
	if (!bmp.begin())
	{
		Log("bmp init failed bye\n");
		exit(-1);
	}

	m.init();
	m.setIntensity(2);
}

// the loop routine runs over and over again forever:
void loop()
{
	// HACK - reset the matrix after every 100 loops - in case 7219s get confused
	if (loop_count > 100)
	{
		m.init();
		m.setIntensity(2);
		loop_count = 0;
	}
	else
	{
		loop_count += 1;
	}

	double avgValue;
	int offset;

	if (millis() - last_reading_millis >= MEASUREMENT_INTERVAL 
		// check if millis has looped back to 0
		|| (millis() < last_upload_millis))
	{
		total_readings += 1;
		total_cpu_temp += readCPUTemp();

		bmptemp = bmp.readTemperature();
		total_bmp085_temp += bmptemp;
		bmppress = bmp.readPressure();
		bmppressK = bmppress / (double)1000;
		total_bmp085_pressure += bmppressK;

		Log("bmp085 temp = %.2f press = %.1f\r\n", bmptemp, bmppressK);

		last_reading_millis = millis();
	}

	m.clear();
	// fill the data buffer with 00s
	//memset(matrixData, 0, sizeof(matrixData));

	// put date and time in buffer
	offset = 0;
	column = 0;

	// write time to LED matrix
	offset += dt.GetTime(matrixData, MATRIX_DATA_LEN - offset, false);
	// add space
	offset += sprintf_s(matrixData + offset, MATRIX_DATA_LEN - offset, " ");
 
	// write date to LED matrix --> uncomment if you want the date to be displayed
	offset += dt.GetDate(matrixData + offset, MATRIX_DATA_LEN - offset);
	// add space
	offset += sprintf_s(matrixData + offset, MATRIX_DATA_LEN - offset, " ");

	// write temperature to LED matrix
	offset += sprintf_s(matrixData + offset, MATRIX_DATA_LEN - offset, "%.1fC", bmptemp);
	// add space
	offset += sprintf_s(matrixData + offset, MATRIX_DATA_LEN - offset, " ");
	// write temperature to LED matrix
	offset += sprintf_s(matrixData + offset, MATRIX_DATA_LEN - offset, "%.1fkPa", bmppressK);

	printString(matrixData);

	// NOTE: the 2nd part of this if statement  -- millis < last_upload_millis -- checks if the millisecond 
		// counter has looped around to 0 (which occurs every 70 days)
	if (!(cosmAPIKey.empty() || cosmFeed.empty()))
	{ 
		if (millis() - last_upload_millis >= COSM_UPLOAD_INTERVAL || (millis() < last_upload_millis))
		{
			if (total_readings > 0)
			{
				avgValue = total_cpu_temp / (double)total_readings;

			
				cosm.SendToCosm(cosmAPIKey.c_str(), stoi(cosmFeed), cosmCPUTempDatastream.c_str(), avgValue);

				avgValue = total_bmp085_temp / (double)total_readings;
				cosm.SendToCosm(cosmAPIKey.c_str(), stoi(cosmFeed), cosmTempDatastream.c_str(), avgValue);

				avgValue = total_bmp085_pressure / (double)total_readings;
				cosm.SendToCosm(cosmAPIKey.c_str(), stoi(cosmFeed), cosmPressureDatastream.c_str(), avgValue);

				total_readings = 0;
				total_cpu_temp = 0;
				total_bmp085_pressure = 0;
				total_bmp085_temp = 0;
				last_upload_millis = millis();
			}
		}
	}
	else
	{

		// wait 1 seconds before scrolling
		delay(1000);
	}

	// scroll until the last column is visible 
	column = column - (8 * matrixCount);
	for (int i = 0; i < column; i++)
	{
		m.shiftLeft(false, false);
		delay(50);
	}

	// delay before clearing the display
	delay(2000);

	
}

void printString(char* s)
{
	// int col = 0;
	while (*s != 0)
	{
		if (*s < 32) continue;
		char c = *s - 32;
		memcpy(buffer, CH + 7 * c, 7);
		m.writeSprite(column, 0, buffer);
		m.setColumn(column + buffer[0], 0);
		column += buffer[0] + 1;
		s++;
	}
}

int readCPUTemp()
{
	//float temperatureInDegreesCelcius = 1.0f;	// Storage for the temperature value

	// reads the analog value from this pin (values range from 0-1023)
	int temperatureInDegreesCelcius = analogRead(-1);

	Log(L"Temperature: %d Celcius\n", temperatureInDegreesCelcius);
	return temperatureInDegreesCelcius;
}

int readIni(char * filename)
{
	string input_str, ini_setting, ini_value;
	ifstream file_in(filename);
	int pos;
	if (!file_in) {
		Log("Ini file not found %s\r\n", filename);
		return 1;
	}
	while (!file_in.eof()) {

		getline(file_in, input_str);
		input_str = delSpaces(input_str);
		// comments start with certain characters - ignore
		if (input_str.find_first_of(";/'") == 0)
		{
			continue;
		}
		pos = input_str.find("=");
		if (pos > 0 && ((pos+1) < input_str.length()))
		{
			ini_setting = input_str.substr(0, pos);
			ini_value = input_str.substr(pos + 1);
			if (ini_setting == "COSM_API_KEY")
			{
				cosmAPIKey = ini_value;
			}
			else if (ini_setting == "COSM_FEED_ID")
			{
				cosmFeed = ini_value;
			} else if (ini_setting == "COSM_CPU_TEMP_DATASTREAM")
			{
				cosmCPUTempDatastream = ini_value;
			} else if (ini_setting == "COSM_TEMP_DATASTREAM")
			{
				cosmTempDatastream = ini_value;
			} else if (ini_setting == "COSM_PRESSURE_DATASTREAM")
			{
				cosmPressureDatastream = ini_value;
			}
			else
			{
				Log("ERROR - Invalid .ini setting %s\n", ini_setting.c_str());
			}
		}

	}
	file_in.close();

	// display the values
	Log("API Key = %s\r\n", cosmAPIKey.c_str());
	Log("Feed ID = %s\r\n", cosmFeed.c_str());
	Log("CPU Temp Datastream = %s\r\n", cosmCPUTempDatastream.c_str());
	Log("Temp Datastream = %s\r\n", cosmCPUTempDatastream.c_str());
	Log("Pressure Datastream = %s\r\n", cosmCPUTempDatastream.c_str());
	return 0;
}

// STL function to  remove all spaces from a string
// take from StackOverflow answer: http://stackoverflow.com/questions/83439/remove-spaces-from-stdstring-in-c
string delSpaces(string &str)
{
	str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
	return str;
}