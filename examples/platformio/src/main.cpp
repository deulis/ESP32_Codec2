/*
 *********************    Codec2 in ESP32    ********************

 This test program implement the encoder and decoder of Codec2
 at 1600bps using LoRa radio.

 Codec 2 is a low-bitrate speech audio codec (speech coding) 
 that is patent free and open source develop by David Grant Rowe.
 http://www.rowetel.com/
 
 This program samples the audio in the transmitter at 8KHz using 
 an ADC a reproduces it in the receiver using a DAC.

 Every 40ms will be generate a new codec2 encoded frame with 8 bytes, 
 then every 5 codec2 frames will be generate a transmission frame.
 In this schema a transmission happened at 200ms intervals, so you 
 have less than 200ms to make the transmission (I'm using 182ms).

 In this implementation the transmission frame has 44 bytes, the 
 first 4 bytes are the header and the others are the voice.
 You can use the header to indicate the address of the transmitter, 
 the address of the desire receiver, etc.

 

 ***********************   W A R N I N G   ********************
 
 This test program DO NOT complies with FCC regulations from USA 
 even not complies with ANATEL from Brazil and I guess do not 
 complies with any regulations from any country.

 To complies with the FCC and orders foreign agencies, normally 
 you need to implement a frequency hopping algorithm that is 
 outside the scope of this test program.

 Please verify your country regulations. You assume all 
 responsibility for the use or misuse of this test program.

 TIP: 
 The challenge of a frequency hopping system is the synchronization, 
 maybe you can use a GPS receiver for synchronization.

 **************************************************************

*/



#include <Arduino.h>
#include <driver/adc.h>

#include <Wire.h>
#include <axp20x.h>

#include <SPI.h>
#include <LoRa.h> //https://github.com/sandeepmistry/arduino-LoRa

#include <codec2.h>				//In the codec2 folder in the library folder
#include <ButterworthFilter.h>	//In the codec2 folder in the library folder
#include <FastAudioFIFO.h>		//In the codec2 folder in the library folder

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     23   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define ADC_PIN ADC1_CHANNEL_6 //ADC 1 canal 0 GPIO36
#define DAC_PIN 14
#define PTT_PIN 38

#define FREQUENCY	915E6
#define ADC_BUFFER_SIZE 320 //40ms of voice in 8KHz sampling frequency
#define ENCODE_FRAME_SIZE 44
#define ENCODE_CODEC2_FRAME_SIZE 8
#define ENCODE_FRAME_HEADER_SIZE 4

AXP20X_Class axp;
#define I2C_SDA         21
#define I2C_SCL         22

FastAudioFIFO audio_fifo;

enum RadioState
{
	radio_standby, radio_rx, radio_tx 
};
volatile RadioState radio_state = RadioState::radio_tx;

int16_t adc_buffer[ADC_BUFFER_SIZE];
int16_t speech[ADC_BUFFER_SIZE];
int16_t output_buffer[ADC_BUFFER_SIZE];
uint8_t transmitBuffer[ADC_BUFFER_SIZE];
unsigned char rx_encode_frame[ENCODE_FRAME_SIZE];
int adc_buffer_index = 0;

//The codec2 
struct CODEC2* codec2_state;

//Implement a high pass 240Hz Butterworth Filter.
ButterworthFilter hp_filter(240, 8000, ButterworthFilter::ButterworthFilter::Highpass, 1);

hw_timer_t* adcTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t codec2HandlerTask;

long last_tick = 0;
long slapsed_in = 0;
uint8_t rx_raw_audio_value = 127;

volatile bool rx_ok = false;
volatile bool tx_ok = false;

int slapsed_encoder, slapsed_decoder, slapsed_tx, slapsed_ack, slapsed_rx, slapsed_rx_ack;
long start_tx;

// OnTxDone event handler
void onTxDone() 
{
	slapsed_tx = millis() - start_tx; //Just for debug
	LoRa.receive(); //The transmission is done, so let's be ready for reception
	tx_ok = true;
}

// onReceive event handler
void onReceive(int packetSize) 
{
	if (packetSize == ENCODE_FRAME_SIZE)// We received a voice packet
	{
		//read the packet
		for (int i = 0; i < packetSize; i++) 
			rx_encode_frame[i] = LoRa.read();

		//Set the state to radio_rx because we are receiving
		radio_state = RadioState::radio_rx;

		// Notify run_codec2 task that we have received a new packet.
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(codec2HandlerTask, &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken)
		{
			portYIELD_FROM_ISR();
		}
	}
	else
	{
		//We don't know what is that, but read all them to empty buffer
		for (int i = 0; i < packetSize; i++)
			LoRa.read();
	}
}

//int16_t 1KHz sine test tone
int16_t Sine1KHz[8] = { -21210 , -30000, -21210, 0 , 21210 , 30000 , 21210, 0 };
int Sine1KHz_index = 0;

void IRAM_ATTR onTimer() {
	portENTER_CRITICAL_ISR(&timerMux); //Enter crital code without interruptions

	if (radio_state == RadioState::radio_tx)
	{
		//Read the ADC and convert is value from (0 - 4095) to (-32768 - 32767)
		adc_buffer[adc_buffer_index++] = (16 * adc1_get_raw(ADC1_CHANNEL_0)) - 32768;

		//If you want to test with a 1KHz tone, comment the line above and descomment the three lines below

		//adc_buffer[adc_buffer_index++] = Sine1KHz[Sine1KHz_index++];
		//if (Sine1KHz_index >= 8)
		//	Sine1KHz_index = 0;

		//When buffer is full
		if (adc_buffer_index == ADC_BUFFER_SIZE) {
			adc_buffer_index = 0;

			slapsed_in = millis() - last_tick; //Just for debug
			last_tick = millis(); //Just for debug

			//Transfer the buffer from adc_buffer to speech buffer
			memcpy((void*)speech, (void*)adc_buffer, 2 * ADC_BUFFER_SIZE); 

			// Notify run_codec2 task that the buffer is ready.
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR(codec2HandlerTask, &xHigherPriorityTaskWoken);
			if (xHigherPriorityTaskWoken)
			{
				portYIELD_FROM_ISR();
			}
		}
	}
	else if (radio_state == RadioState::radio_rx)
	{
		int16_t v;

		//Get a value from audio_fifo and convert it to 0 - 255 to play it in the ADC
		//If none value is available the DAC will play the last one that was read, that's
		//why the rx_raw_audio_value variable is a global one.
		if (audio_fifo.get(&v))
			rx_raw_audio_value = (uint8_t)((v + 32768) / 256);

		//Play
		dacWrite(DAC_PIN, rx_raw_audio_value);
	}
	portEXIT_CRITICAL_ISR(&timerMux); // exit critical code
}

void run_codec2(void* parameter)
{
	//Init codec2
	codec2_state = codec2_create(CODEC2_MODE_1600);
	codec2_set_lpc_post_filter(codec2_state, 1, 0, 0.8, 0.2);

	long start_encoder, start_decoder; //just for debug
	
	unsigned char tx_encode_frame[ENCODE_FRAME_SIZE];
	int tx_encode_frame_index = 0;

	//Header, you have 4 bytes of header in each frame, yo can use it for you protocol implementation
	// for instace indicate address of the transmiter and address of the desire receiver, etc.
	tx_encode_frame[0] = 0x00;
	tx_encode_frame[1] = 0x00;
	tx_encode_frame[2] = 0x00;
	tx_encode_frame[3] = 0x00;
	
	RadioState last_state = RadioState::radio_standby;
	while (1)
	{
		//Wait until be notify or 1 second
		uint32_t tcount = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));

		if (tcount != 0) //if the task was notified! 
		{
			//Init the tx_encode_frame_index if a trasmition start
			if (radio_state != last_state)
			{
				if (radio_state == RadioState::radio_tx)
				{
					tx_encode_frame_index = ENCODE_FRAME_HEADER_SIZE;
				}
				last_state = radio_state;
			}

			
			if (radio_state == RadioState::radio_tx) //Trasnmitting
			{
				start_encoder = millis(); //Just for debug

				//Apply High Pass Filter
				for (int i = 0; i < ADC_BUFFER_SIZE; i++)
					speech[i] = (int16_t)hp_filter.Update((float)speech[i]);

				//encode the 320 bytes(40ms) of speech frame into 8 bytes
				codec2_encode(codec2_state, tx_encode_frame + tx_encode_frame_index, speech);	

				//increment the pointer where the encoded frame must be saved
				tx_encode_frame_index += ENCODE_CODEC2_FRAME_SIZE; 

				slapsed_encoder = millis() - start_encoder; //Just for debug

				//If it is the 5th time then we have a ready trasnmission frame
				if (tx_encode_frame_index == ENCODE_FRAME_SIZE)
				{
					start_tx = millis(); //Just for debug
					tx_encode_frame_index = ENCODE_FRAME_HEADER_SIZE;

					//Transmit it
					LoRa.beginPacket();
					LoRa.write(tx_encode_frame, ENCODE_FRAME_SIZE);
					LoRa.endPacket(true);
				}
			}
			if (radio_state == RadioState::radio_rx) //Receiving
			{
				start_decoder = millis(); //Just for debug

				//Make a cycle to get each codec2 frame from the received frame
				for (int i = ENCODE_FRAME_HEADER_SIZE; i < ENCODE_FRAME_SIZE; i += ENCODE_CODEC2_FRAME_SIZE)
				{
					//Decode the codec2 frame
					codec2_decode(codec2_state, output_buffer, rx_encode_frame + i);
					
					// Add to the audio buffer the 320 samples resulting of the decode of the codec2 frame.
					for (int g = 0; g < ADC_BUFFER_SIZE; g++)
						audio_fifo.put(output_buffer[g]);
				}

				slapsed_decoder = millis() - start_decoder; //Just for debug
				rx_ok = true;				
			}
		}
	}
}

void setup() {
	Serial.begin(115200); //Just for debug

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin PASS");
  } else {
    Serial.println("AXP192 Begin FAIL");
  }
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS main power
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // provides power to GPS backup battery
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // enables power to ESP32 on T-beam
  axp.setPowerOutPut(AXP192_DCDC3, AXP202_ON); // I foresee similar benefit for restting T-watch 
                                                 // where ESP32 is on DCDC3 but remember to change I2C pins and GPS pins!
	SPI.begin(SCK, MISO, MOSI, SS);
	LoRa.setPins(SS, RST, DI0);
	if (!LoRa.begin(FREQUENCY)) 
	{
		Serial.println("Starting LoRa failed!");
		while (1);
	}
	LoRa.setSpreadingFactor(9);
	LoRa.setSignalBandwidth(250E3);
	LoRa.setCodingRate4(7); // 4/7
	LoRa.setPreambleLength(6);
	LoRa.setSyncWord(0x12);
	LoRa.enableCrc();
	LoRa.onTxDone(onTxDone);
	LoRa.onReceive(onReceive);


	adc1_config_width(ADC_WIDTH_12Bit);
	adc1_config_channel_atten(ADC_PIN, ADC_ATTEN_DB_6); //ADC 1 canal 0 GPIO36

	//Start the task that run the coder and decoder
	xTaskCreate(&run_codec2, "codec2_task", 30000, NULL, 5, &codec2HandlerTask);

	//Start a timer at 8kHz to sample the ADC and play the audio on the DAC.
	adcTimer = timerBegin(3, 500, true); // 80 MHz / 500 = 160KHz MHz hardware clock
	timerAttachInterrupt(adcTimer, &onTimer, true); // Attaches the handler function to the timer 
	timerAlarmWrite(adcTimer, 20, true); // Interrupts when counter == 20, 8.000 times a second
	timerAlarmEnable(adcTimer); //Activate it

	last_tick = millis(); //Just for debug

	//Configure PTT input
	pinMode(PTT_PIN, INPUT_PULLUP);

	//Set state 
	radio_state = RadioState::radio_rx;
	
	//Let's start receiving
	LoRa.receive();
}

int tx_ok_counter = 0; //Just for debug
int rx_ok_counter = 0; //Just for debug
void loop() {

	if (digitalRead(PTT_PIN) == LOW)
	{
		radio_state = RadioState::radio_tx;
	}
	else if(tx_ok)
	{
		radio_state = RadioState::radio_rx;
	}


	//Some DEBUG stuffs, you can remove it if you want.
	if (rx_ok)
	{
		Serial.print(rx_ok_counter);
		Serial.print(" Dec=");
		Serial.println(slapsed_decoder);
		rx_ok_counter++;
		rx_ok = false;
	}

	if (tx_ok)
	{
		Serial.print(tx_ok_counter);
		Serial.print(" Enc=");
		Serial.print(slapsed_encoder);
		Serial.print(" Tx=");
		Serial.println(slapsed_tx);		
		tx_ok_counter++;
		tx_ok = false;
	}

	

	delay(1);//At least 1ms please!
}
