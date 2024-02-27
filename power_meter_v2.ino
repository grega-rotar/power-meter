// vključitev lokalnih header datotek
#include "secrets.h"
#include "mpu6050_const.h"
#include "bike_details.h"
#include "const.h"

// za BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "BLE_const.h"
BLEServer *pServer = NULL;
BLECharacteristic *pPMCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
class MyServerCallbacks : public BLEServerCallbacks
{
	void onConnect(BLEServer *pServer)
	{
		deviceConnected = true;
	};

	void onDisconnect(BLEServer *pServer)
	{
		deviceConnected = false;
	}
};

// knjižnice
#include <Wire.h>
#include "HX711.h"
#include <WiFi.h>
#include <PubSubClient.h>

// Wi-Fi spremenljivke
const char *ssid = W_SSID;
const char *pass = W_PASS;
WiFiClient wifiClient;

// MQTT Broker
const char *mqtt_broker = MQTT_BROKER;
const int mqtt_port = MQTT_PORT;
const char *topic = "bike-gen/mocKolesarja-meter";
const char *mqtt_username = MQTT_USER;
const char *mqtt_password = MQTT_PASS;
PubSubClient client(wifiClient);

// MPU-6050
// pospeškomer
int16_t accelX, accelY, accelZ;
double gSilaX, gSilaY, gSilaZ;
// žiroskop
int16_t ziroX, ziroY, ziroZ;
double rotX, rotY, rotZ;

// kalibracijske številke
typedef float Matrix3x3[3][3];
// 3x3 martika pridobljena z programom magneto 1.2
// NE SPREMINJAJ VREDNOSTI
Matrix3x3 mpuCalibMatrix = {
	{0.994508, 0.001218, 0.003548},
	{0.001218, 0.994691, -0.000662},
	{0.003548, -0.000662, 0.996563}};

// HX711
#define SCALE_DOUT_PIN 16 // Replace with your chosen GPIO pin
#define SCALE_SCK_PIN 4	  // Replace with your chosen GPIO pin
HX711 scale;
long surovaTeza;

#define THRESHOLD 0.01
struct VrednostiMeritev
{
	// FIXME:  povprecjeTezaPozitivne in povprecjeTezaNegativne se updata samo ko klicemo funkciji za pridobitev povprecja
	double vsotaTezaPozitivne;
	double stevecTezaPozitivne;
	double povprecjeTezaPozitivne;

	double vsotaTezaNegativne;
	double stevecTezaNegativne;
	double povprecjeTezaNegativne;

	bool jeTezaNaPozitivniStrani;

	double vsotaKotnaHitrostDEG;
	double stevecKotnaHitrostDEG;
	PubSubClient& mqttClient;

	VrednostiMeritev(PubSubClient& client)
		: vsotaTezaPozitivne(0), stevecTezaPozitivne(0), povprecjeTezaPozitivne(0),
		  vsotaTezaNegativne(0), stevecTezaNegativne(0), povprecjeTezaNegativne(0), jeTezaNaPozitivniStrani(true), vsotaKotnaHitrostDEG(0), stevecKotnaHitrostDEG(0), mqttClient(client) {}

	// funkcija, ki doda meritev teze glede na predznak
	void dodajMeritevTeza(double vrednost)
	{
		bool vrednostPredznak = jePredznakPozitiven(vrednost);
		switch (vrednostPredznak == jeTezaNaPozitivniStrani)
		{
		// predznak enak
		// dodajanje k trenutnim meritvam
		case true:
			dodajTezaGledePredznak(vrednost);
			Serial.println(vrednost);
			break;
		// v tem primeru se je predznak spremenil
		// prehod čez vrednost 0
		case false:
			// če je meritev večja od THRESHOLD, potem lahko shranimo drugača se šteje, kot da je meritev napačna
			if (abs(vrednost) >= THRESHOLD)
			{
				// jeTezaNaPozitivniStrani se bo spremenil saj začnemo pridobivati meritve iz drugega predznaka
				// s switch izpisemu tista povprecja / vrednosti, ki smo jih sedaj merili
				switch (jeTezaNaPozitivniStrani)
				{
				// predznak se bo spremenil na negativnega
				// izracun povprecja pozitivnih vrednosti
				case true:
					povprecjeTezaPozitivne = pridobiPovprecjeTezaPozitivne(true);
					break;
				// predznak se bo spremenil na pozitivnega
				// izracun povprecja negativnih vrednosti
				case false:
					povprecjeTezaNegativne = pridobiPovprecjeTezaNegativne(true);
					break;
				}
				// funkcije, ki se zgodijo ob menjavi predznaka
				double mocKolesarja = pridobiMocKolesarja();
				mqttClient.publish(topic, String(mocKolesarja).c_str());
				// jeTezaNaPozitivniStrani se spremeni v predznak trenutnih meritev
				jeTezaNaPozitivniStrani = jePredznakPozitiven(vrednost);
				// dodana je trenutna vrednost k meritvam
				dodajTezaGledePredznak(vrednost);
				// izpis trenutne meritve
				Serial.println(vrednost);
			}
		}
	}

	// vrne moc kolesarja v W
	double pridobiMocKolesarja()
	{
		// povprecna kotna hitrost v deg/s
		double povprecnaKotnaHitrostDEG = pridobiPovprecjeKotnaHitrostDEG(true);
		// povprecna kotna hitrost v rad/s
		double kotnaHitrostRAD = (povprecnaKotnaHitrostDEG * PI) / 180.0;
		double povprecnaTezaKG = povprecjeTezaNegativne + povprecjeTezaPozitivne;
		double povprecnaSilaNaPedalo = povprecnaTezaKG * G_ACC;
		double povprecenNavor = povprecnaSilaNaPedalo * GONILKA_ROCICA_M;
		double mocKolesarja = povprecenNavor * kotnaHitrostRAD;
		return mocKolesarja;
	}

	// doda tezo k vsoti tez
	void dodajTezaGledePredznak(double vrednost)
	{
		bool vrednostPredznak = jePredznakPozitiven(vrednost);
		switch (vrednostPredznak)
		{
		case true:
			vsotaTezaPozitivne += vrednost;
			stevecTezaPozitivne++;
			break;
		case false:
			vsotaTezaNegativne += vrednost;
			stevecTezaNegativne++;
		}
	}

	// prišteje kotno hitrost vsoti
	void dodajKotnaHitrostDEG(double kotnaHitrostDEG)
	{
		vsotaKotnaHitrostDEG += kotnaHitrostDEG;
		stevecKotnaHitrostDEG++;
	}

	// vrne povprecje kotne hitrosti v DEG/s
	double pridobiPovprecjeKotnaHitrostDEG(bool reset)
	{
		double povprecje = vsotaKotnaHitrostDEG / stevecKotnaHitrostDEG;
		if (reset)
		{
			vsotaKotnaHitrostDEG = 0;
			stevecKotnaHitrostDEG = 0;
		}
		return povprecje;
	}

	// vrne true, če je predznak pozitiven
	bool jePredznakPozitiven(double stevilo)
	{
		return (stevilo >= 0);
	}

	// vrne povprecje pozitivnih tež, reset parameter resetira vsoto in stevec
	double pridobiPovprecjeTezaPozitivne(bool reset)
	{
		double povprecje = 0;
		povprecje = vsotaTezaPozitivne / stevecTezaPozitivne;
		if (reset)
		{
			vsotaTezaPozitivne = 0;
			stevecTezaPozitivne = 0;
		}
		povprecjeTezaPozitivne = povprecje;
		return povprecje;
	}

	// vrne povprecje negativnih tež
	// reset parameter resetira vsoto in stevec
	double pridobiPovprecjeTezaNegativne(bool reset)
	{
		double povprecje = 0;
		povprecje = vsotaTezaNegativne / stevecTezaNegativne;
		if (reset)
		{
			vsotaTezaNegativne = 0;
			stevecTezaNegativne = 0;
		}
		povprecjeTezaNegativne = povprecje;
		return povprecje;
	}

	// ponastavi vsote in števce na 0
	void ponastavi()
	{
		vsotaTezaPozitivne = 0;
		stevecTezaPozitivne = 0;
		vsotaTezaNegativne = 0;
		stevecTezaNegativne = 0;
	}
};
// KONEC STRUCT VrednostiMeritev

void setup()
{
	Serial.begin(115200);

	// BLE
	BLEDevice::init(BLE_DEVICE_NAME);
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());

	// PM stands for mocKolesarja meter
	BLEService *pPMService = pServer->createService(PM_SERVICE_UUID);

	pPMCharacteristic = pPMService->createCharacteristic(
		PM_CHARACTERISTIC_UUID,
		BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

	pPMCharacteristic->addDescriptor(new BLE2902());
	pPMService->start();
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(PM_SERVICE_UUID);
	pAdvertising->setScanResponse(false);
	pAdvertising->setMinPreferred(0x0);
	BLEDevice::startAdvertising();
	// end of BLE

	// MPU-6050
	Wire.begin(); // defaultni 21 (SDA) in 22 (SCL) drugace dodaj kot parameter
	setupMpu();

	// hx711
	scale.begin(SCALE_DOUT_PIN, SCALE_SCK_PIN);
	// scale.set_offset(-887600);

	WiFi.begin(ssid, pass);
	while (WiFi.status() != WL_CONNECTED)
	{
		Serial.print(".");
		delay(500);
	}

	Serial.println("Connected to WiFi");
	// connecting to a MQTT broker START
	client.setServer(mqtt_broker, mqtt_port);

	// loop until client is connected
	while (!client.connected())
	{
		String client_id = "power_meter_" + generateRandomClientId();

		if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
		{
			Serial.println("MQTT broker connected");
		}
		else
		{
			Serial.print("failed with state ");
			Serial.print(client.state());
			delay(2000);
		}
	}
}

// globalne spremenljivke za
double trenutnaTezaKG;
double trenutnaKotnaHitrostDEG;

// inicalizacija VrednostiMeritev
VrednostiMeritev vrednostiMeritev(client);

void loop()
{

	// posodobi vrednosti pospeškomera
	pridobiAccelVrednosti();
	// posodobi vrednost žiroskopa
	pridobiZiroVrednosti();
	// posodobi trenutno vrednost teze
	pridobiSurovoTezo();

	trenutnaTezaKG = ((surovaTeza - (-107400.0)) / -74170.0) - 4.5;
	trenutnaKotnaHitrostDEG = sqrt(pow(rotX, 2) + pow(rotY, 2) + pow(rotZ, 2));
	vrednostiMeritev.dodajKotnaHitrostDEG(trenutnaKotnaHitrostDEG);
	vrednostiMeritev.dodajMeritevTeza(trenutnaTezaKG);

	delay(15);
}

// funkcije za HX711
// WARNING: be careful when using because it will not neccesseraly update
void pridobiSurovoTezo()
{
	if (scale.is_ready())
	{
		surovaTeza = scale.read_average(1);
	}
}

// funkcije za MPU-6050
// mpu6050_const.h vsebuje veliko spremenljivk za to

// functions for MPU-6050
// mpu6050_const.h contains lots of variables for this
void setupMpu()
{
	// osnovna konfiguracija
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x6B);		// register za upravljanje napajanja
	Wire.write(0b00000000); // nastavi SLEEP register na 0 (glej 4.28)
	Wire.endTransmission();

	// konfiguracija žiroskopa
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x1B);
	// Bit 3 in 4 za konfiguracijo žiroskopa (glej 4.4)
	// 00 (0) +/- 250deg/s
	// 01 (1) +/- 500deg/s
	// 10 (2) +/- 1000deg/s
	// 11 (3) +/- 2000deg/s
	uint8_t gyro_nastavitev;
	// s spreminjanjem 3 in 4 bita lahko nastavimo različno občutljivost senzorja
	switch (FS_SEL)
	{
	case 0:
		gyro_nastavitev = 0b00000000; // nastavljen na +/- 250deg/sec
		break;
	case 1:
		gyro_nastavitev = 0b00001000; // nastavljen na +/- 500deg/sec
		break;
	case 2:
		gyro_nastavitev = 0b00010000; // nastavljen na +/- 1000deg/sec
		break;
	case 3:
		gyro_nastavitev = 0b00011000; // nastavljen na +/- 2000deg/sec
		break;
	}
	Wire.write(gyro_nastavitev);
	Wire.endTransmission();

	// konfiguracija pospeškomera
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x1C);
	// 00 (0) +/- 2g
	// 01 (1) +/- 4g
	// 10 (2) +/- 8g
	// 11 (3) +/- 16g
	uint8_t accel_nastavitev;
	// s spreminjanjem 3 in 4 bita lahko nastavimo različno občutljivost senzorja
	switch (AFS_SEL)
	{
	case 0:
		accel_nastavitev = 0b00000000; // nastavljen na +/- 2g
		break;
	case 1:
		accel_nastavitev = 0b00001000; // nastavljen na +/- 4g
		break;
	case 2:
		accel_nastavitev = 0b00010000; // nastavljen na +/- 8g
		break;
	case 3:
		accel_nastavitev = 0b00011000; // nastavljen na +/- 16g
		break;
	}
	Wire.write(accel_nastavitev);
	Wire.endTransmission();
}

void pridobiAccelVrednosti()
{
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x3B); // izbran začetni register
	Wire.endTransmission();
	Wire.requestFrom(MPU_ADDR, 6); // zahteva accel registre (3B - 40)
	while (Wire.available() < 6)
		; // program caka da je na voljo vseh 6 bytov
	accelX = (Wire.read() << 8) | Wire.read();
	accelY = (Wire.read() << 8) | Wire.read();
	accelZ = (Wire.read() << 8) | Wire.read();

	procesirajAccelVrednosti();
}

// FIXME: funkcija shranjuje umerjene vrednosti znotraj globalnih gSilaX,Y,Z, zato ne morete dostopati do nekalibriranih podatkov (lahko, vendar samo iz surovega)
void procesirajAccelVrednosti()
{
	// LSB/g glej poglavje 4.17
	// FIXME: vrednost je določena le ko je accel nastavljen na 2G
	// kalibracija z magneto 1.2 100 podatkov
	// to kar počne je v bistvu samo delitev pristranskosti, tako da se ujema s trenutno nastavitvijo deg/sec
	gSilaX = (accelX - ((BIAS_ACCEL_X) / (LSB_G_CALIBRATED / LSB_G)));
	gSilaY = (accelY - ((BIAS_ACCEL_Y) / (LSB_G_CALIBRATED / LSB_G)));
	gSilaZ = (accelZ - ((BIAS_ACCEL_Z) / (LSB_G_CALIBRATED / LSB_G)));

	gSilaX = mpuCalibMatrix[0][0] * gSilaX + mpuCalibMatrix[0][1] * gSilaY + mpuCalibMatrix[0][2] * gSilaZ;
	gSilaY = mpuCalibMatrix[1][0] * gSilaX + mpuCalibMatrix[1][1] * gSilaY + mpuCalibMatrix[1][2] * gSilaZ;
	gSilaZ = mpuCalibMatrix[2][0] * gSilaX + mpuCalibMatrix[2][1] * gSilaY + mpuCalibMatrix[2][2] * gSilaZ;

	gSilaX = gSilaX / LSB_G;
	gSilaY = gSilaY / LSB_G;
	gSilaZ = gSilaZ / LSB_G;
}

void pridobiZiroVrednosti()
{
	Wire.beginTransmission(MPU_ADDR); // I2C naslov MPU
	Wire.write(0x43);				  // izbran začeten register
	Wire.endTransmission();
	Wire.requestFrom(0b1101000, 6); // zahteva ziro registere (43 - 48)
	while (Wire.available() < 6)
		; // program caka da je na voljo vseh 6 bytov
	ziroX = Wire.read() << 8 | Wire.read();
	ziroY = Wire.read() << 8 | Wire.read();
	ziroZ = Wire.read() << 8 | Wire.read();
	procesirajZiroVrednosti();
}

void procesirajZiroVrednosti()
{
	// LSB/deg/s glej poglavje 4.19
	// to kar počne je v bistvu samo delitev pristranskosti, tako da se ujema s trenutno nastavitvijo deg/sec
	rotX = (ziroX - ((BIAS_GYRO_X) / (LSB_DEG_CALIBRATED / LSB_DEG)));
	rotY = (ziroY - ((BIAS_GYRO_Y) / (LSB_DEG_CALIBRATED / LSB_DEG)));
	rotZ = (ziroZ - ((BIAS_GYRO_Z) / (LSB_DEG_CALIBRATED / LSB_DEG)));

	// FIXME: ne vem, če je ta matrika najboljša za podatke žiroskopa
	rotX = mpuCalibMatrix[0][0] * rotX + mpuCalibMatrix[0][1] * rotY + mpuCalibMatrix[0][2] * rotZ;
	rotY = mpuCalibMatrix[1][0] * rotX + mpuCalibMatrix[1][1] * rotY + mpuCalibMatrix[1][2] * rotZ;
	rotZ = mpuCalibMatrix[2][0] * rotX + mpuCalibMatrix[2][1] * rotY + mpuCalibMatrix[2][2] * rotZ;

	rotX = rotX / LSB_DEG;
	rotY = rotY / LSB_DEG;
	rotZ = rotZ / LSB_DEG;
}


// FIXME: ko odpravis mqtt lahko izbrišeš
char generateRandomChar()
{
	const char charset[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
	const int charsetSize = sizeof(charset) - 1;

	// Generate a random index within the character set
	int randomIndex = rand() % charsetSize;

	// Return the random character
	return charset[randomIndex];
}

// FIMX: ko odpravis mqtt lahko izbrišeš
String generateRandomClientId()
{
	String clientId = "";
	for (int i = 0; i < 8; ++i)
	{
		clientId += generateRandomChar();
	}
	return clientId;
}