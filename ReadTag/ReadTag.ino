#include <SPI.h>
#include <RFID.h>

#define SPI_MOSI 51
#define SPI_MISO 50
#define SPI_SCLK 52
#define SPI_SS   53
#define MFRC522_RSTPD 49

RFID rfid(SPI_SS, MFRC522_RSTPD);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  
	SPI.begin();
	SPI.beginTransaction(SPISettings(10000000L, MSBFIRST, SPI_MODE3));

	rfid.begin();

	Serial.begin(9600);
	while(!Serial);
}

static uint8_t status;
static uint16_t card_type;
static uint8_t sn[MAXRLEN], snBytes;

void loop()
{
	delay(200);
	if ((status = rfid.findTag(&card_type)) == STATUS_OK) {
    digitalWrite(LED_BUILTIN, HIGH);
		Serial.print("OK! ");
		Serial.println(card_type);
		if ((status = rfid.readTagSN(sn, &snBytes)) == STATUS_OK) {
			for (int i = 0; i < snBytes; ++i)
				Serial.print(sn[i], HEX);
			Serial.println();
			rfid.piccHalt();
		}
	}else{
		Serial.println("No tag.");
    digitalWrite(LED_BUILTIN, LOW); 
	}
}
