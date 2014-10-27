// Do not remove the include below
#include "HelenaArduinoTest.h"

#include <Arduino.h>
#include <SPI.h>
#include "services.h"
#include <lib_aci.h>
#include <aci_setup.h>
extern "C"{
#include "buffer_h.h"
};
static services_pipe_type_mapping_t services_pipe_type_mapping[NUMBER_OF_PIPES] =
		SERVICES_PIPE_TYPE_MAPPING_CONTENT;

static hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM =
		SETUP_MESSAGES_CONTENT;

static aci_state_t aci_state;
static hal_aci_evt_t aci_data;
static bool radio_ack_pending = false;
static bool timing_change_done = false;
bool data_tx_state = false;
/*** FUNC
 Name:           Timer1start
 Function:       Start timer 1 to interrupt periodically. Call this from
 the Arduino setup() function.
 Description:    The pre-scaler and the timer count divide the timer-counter
 clock frequency to give a timer overflow interrupt rate:

 Interrupt rate =  16MHz / (prescaler * (255 - TCNT2))

 TCCR2B[b2:0]   Prescaler    Freq [KHz], Period [usec] after prescale
 0x0            (TC stopped)     0         0
 0x1                1        16000.        0.0625
 0x2                8         2000.        0.500
 0x3               32          500.        2.000
 0x4               64          250.        4.000
 0x5              128          125.        8.000
 0x6              256           62.5      16.000
 0x7             1024           15.625    64.000
 FUNC ***/

void Timer1start() {

	// Setup Timer1 overflow to fire every 4000ms
	//   period [sec] = (1 / f_clock [sec]) * prescale * (count)
	//                  (1/16000000)  * 1024 * (count) = 4000 ms

	TCCR1B = 0x00;        // Disable Timer1 while we set it up

	TCNT1H = 11;          // Approx 4000ms when prescaler is set to 1024
	TCNT1L = 0;
	TIFR1 = 0x00;        // Timer1 INT Flag Reg: Clear Timer Overflow Flag
	TIMSK1 = 0x01;        // Timer1 INT Reg: Timer1 Overflow Interrupt Enable
	TCCR1A = 0x00;        // Timer1 Control Reg A: Wave Gen Mode normal
	TCCR1B = 0x05;        // Timer1 Control Reg B: Timer Prescaler set to 1024
}

void Timer1stop() {
	TCCR1B = 0x00;
	TIMSK1 = 0x00;
}

typedef struct ThatDevicePacket {
	uint8_t thisDevice[6];
	uint8_t thatDevice[6];
	uint32_t time;
};

typedef struct ObservationPacket {
	uint8_t SizeQueue;
	uint8_t squall[6];
	uint8_t observedMAC[6];
	uint16_t observedManufactor;
	uint32_t time;
};

void randomNum(void) {
//	if (lib_aci_is_pipe_available(&aci_state, PIPE_READDEVICE_DATAAVAILABLE_TX)) {
//		uint8_t data = 1;
//
//		lib_aci_send_data(PIPE_READDEVICE_DATAAVAILABLE_TX, (uint8_t*) &data,
//				sizeof(uint8_t));
//		Serial.println(F("Notifying"));
//
//		aci_state.data_credit_available--;
//	}
	//Serial.println(F("random"));
}


/*** FUNC
 Name:       Timer1 ISR
 Function:   Handles the Timer1-overflow interrupt
 FUNC ***/
ISR(TIMER1_OVF_vect) {
	randomNum();
	TCNT1H = 11;    // Approx 4000 ms - Reload
	TCNT1L = 0;
	TIFR1 = 0x00;    // timer1 int flag reg: clear timer overflow flag
}
;

/* Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line) {
	Serial.print("ERROR ");
	Serial.print(file);
	Serial.print(": ");
	Serial.print(line);
	Serial.print("\n");
	while (1)
		;
}

void setup(void) {
	Serial.begin(115200);

	Serial.println(F("Arduino setup: Glucose measurement"));

	/**
	 Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
	 */
	if (NULL != services_pipe_type_mapping) {
		aci_state.aci_setup_info.services_pipe_type_mapping =
				&services_pipe_type_mapping[0];
	} else {
		aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
	}
	aci_state.aci_setup_info.number_of_pipes = NUMBER_OF_PIPES;
	aci_state.aci_setup_info.setup_msgs = setup_msgs;
	aci_state.aci_setup_info.num_setup_msgs = NB_SETUP_MESSAGES;

	//Tell the ACI library, the MCU to nRF8001 pin connections
	aci_state.aci_pins.board_name = BOARD_DEFAULT; //REDBEARLAB_SHIELD_V1_1 See board.h for details
	aci_state.aci_pins.reqn_pin = 9;
	aci_state.aci_pins.rdyn_pin = 8;
	aci_state.aci_pins.mosi_pin = MOSI;
	aci_state.aci_pins.miso_pin = MISO;
	aci_state.aci_pins.sck_pin = SCK;

	aci_state.aci_pins.spi_clock_divider = SPI_CLOCK_DIV8; //SPI_CLOCK_DIV8  = 2MHz SPI speed
														   //SPI_CLOCK_DIV16 = 1MHz SPI speed

	aci_state.aci_pins.reset_pin = 4;
	aci_state.aci_pins.active_pin = UNUSED;
	aci_state.aci_pins.optional_chip_sel_pin = UNUSED;

	aci_state.aci_pins.interface_is_interrupt = false;
	aci_state.aci_pins.interrupt_number = UNUSED;

	/** We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
	 *  and initialize the data structures required to setup the nRF8001
	 */
	//The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
	lib_aci_init(&aci_state, false);
	//init_buffer(30);

}

void aci_loop() {
	static bool setup_required = false;

	// We enter the if statement only when there is a ACI event available to be processed
	if (lib_aci_event_get(&aci_state, &aci_data)) {
		aci_evt_t * aci_evt;
		aci_evt = &aci_data.evt;

		//@todo change this so that the commands and events can be processed here in a switch case instead of callbacks
		//hal_aci_tl_msg_rcv_hook(&aci_data);
		switch (aci_evt->evt_opcode) {
		case ACI_EVT_DEVICE_STARTED: {
			aci_state.data_credit_available =
					aci_evt->params.device_started.credit_available;
			switch (aci_evt->params.device_started.device_mode) {
			case ACI_DEVICE_SETUP:
				/**
				 When the device is in the setup mode
				 */
				aci_state.device_state = ACI_DEVICE_SETUP;
				Serial.println(F("Evt Device Started: Setup"));
				setup_required = true;
				break;

			case ACI_DEVICE_STANDBY:
				aci_state.device_state = ACI_DEVICE_STANDBY;
				Serial.println(F("Evt Device Started: Standby"));
				if (aci_evt->params.device_started.hw_error) {
					delay(20); //Magic number used to make sure the HW error event is handled correctly.
				} else {
					Timer1start();
					lib_aci_connect(30/* in seconds */,
							0x0100 /* advertising interval 100ms*/);
					Serial.println(F("Advertising started"));
					Serial.print(F("Connection interval: "));
					Serial.print(
							aci_evt->params.timing.conn_rf_interval * 1.25
									* 100, DEC);

					Serial.println(F(" ms"));
				}
				break;
			}
		}
			break; //ACI Device Started Event

		case ACI_EVT_CMD_RSP:
			//If an ACI command response event comes with an error -> stop
			if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status) {
				//ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
				//TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
				//all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command

				Serial.print(F("ACI Status of ACI Evt Cmd Rsp 0x"));
				Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
				Serial.print(F("ACI Command 0x"));
				Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
				Serial.println(
						F(
								"Evt Cmd respone: Error. Arduino is in an while(1); loop"));
				while (1)
					;
			}

			break;

		case ACI_EVT_PIPE_STATUS:
			Serial.println(F("ACI_EVT_PIPE_STATUS"));

			if (lib_aci_is_pipe_available(&aci_state,
					PIPE_READDEVICE_DATAAVAILABLE_TX)
					&& (false == timing_change_done)) {
				Serial.println(
						F("PIPE_READDEVICE_DATAAVAILABLE_TX && Timing change"));
				lib_aci_change_timing_GAP_PPCP();
				timing_change_done = true;
				lib_aci_send_data(PIPE_READDEVICE_DATAAVAILABLE_TX, (uint8_t*) &data,
						sizeof(uint8_t));
				Serial.println(F("Notifying"));

				aci_state.data_credit_available--;
			}
			if (lib_aci_is_pipe_available(&aci_state, PIPE_READDEVICE_OBSERVATION_SET))
            {
              Serial.println(F("Sending buffer data"));
              data_tx_state = true;
            }
			break;

		case ACI_EVT_TIMING:
			/*
			 Link timing has changed.
			 */
			Serial.print(F("Timing changed. New connection interval: "));
			Serial.print(aci_evt->params.timing.conn_rf_interval * 1.25, 0);
			Serial.println(F(" ms"));
			break;

		case ACI_EVT_CONNECTED:

			radio_ack_pending = false;
			aci_state.data_credit_available = aci_state.data_credit_total;
			timing_change_done = false;
			Serial.println(F("ACI_EVT_CONNECTED "));
			break;

		case ACI_EVT_DATA_CREDIT:
			aci_state.data_credit_available = aci_state.data_credit_available
					+ aci_evt->params.data_credit.credit;
			Serial.print("ACI_EVT_DATA_CREDIT     ");
			Serial.print("Data Credit available: ");
			Serial.println(aci_state.data_credit_available, DEC);
			break;

		case ACI_EVT_PIPE_ERROR:

			if (ACI_STATUS_ERROR_PEER_ATT_ERROR
					!= aci_evt->params.pipe_error.error_code) {
				aci_state.data_credit_available++;
			}
			radio_ack_pending = false;
			break;

		case ACI_EVT_DISCONNECTED:

			if (ACI_STATUS_ERROR_ADVT_TIMEOUT
					== aci_evt->params.disconnected.aci_status) {
				Serial.println(F("Evt Disconnected -> Advertising timed out"));
			} else {
				Serial.println(F("Evt Disconnected -> Link lost."));
			}
			lib_aci_connect(180/* in seconds */,
					0x0050 /* advertising interval 50ms*/);
			Serial.println(F("Advertising started"));
			break;

		case ACI_EVT_HW_ERROR:
			Serial.println(F("HW error: "));
			Serial.println(aci_evt->params.hw_error.line_num, DEC);

			for (uint8_t counter = 0; counter <= (aci_evt->len - 3);
					counter++) {
				Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
			}
			Serial.println();
			Timer1start();
			lib_aci_connect(30/* in seconds */,
					0x0100 /* advertising interval 100ms*/);
			Serial.println(F("Advertising started"));
			break;

		default:
			Serial.println(F("Default opt code"));
			Serial.println(aci_evt->evt_opcode, DEC);
			break;
		}
	} else {
		// If No event in the ACI Event queue and No event in the ACI Command queue
		// Arduino can go to sleep
	}

	if (setup_required) {
		if (SETUP_SUCCESS == do_aci_setup(&aci_state)) {
			setup_required = false;
		}
	}
}

void loop() {
	aci_loop();
	  if ((data_tx_state == true) && (aci_state.data_credit_available > 0))
	  {
			ObservationPacket obsPkt={
								 1 ,
								{(uint8_t)random(1,3)},
								{(uint8_t)random(10,20)},
								(uint16_t)random(0,397),
								millis()
						};

			lib_aci_set_local_data(&aci_state, PIPE_READDEVICE_OBSERVATION_SET, (uint8_t*) &obsPkt,
							sizeof(ObservationPacket));
			aci_state.data_credit_available--;
	  }
}
