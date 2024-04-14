#include <Bluepad32.h>
#include <HardwareSerial.h>
// #include <BluetoothSerial.h>



// BluetoothSerial serial_bt;
HardwareSerial main_serial_port(2);
GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

void onConnectedGamepad(GamepadPtr gp) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == nullptr) {
      Serial.printf("CALLBACK: Gamepad is connected, index=%d\n", i);

      GamepadProperties properties = gp->getProperties();
      Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myGamepads[i] = gp;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Gamepad connected, but could not found empty slot");
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == gp) {
      Serial.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
      myGamepads[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
  }
}
/*TODO: Receive data for the live follower from bluetooth and process it*/
void receive_live_follower_data() {
}
/*TODO: Send data to the mcu for following (coordinates and speeds)*/
void send_live_follower_data() {
}
/*TODO: Process data form GUI*/
void live_follower() {
}




/*TODO: Send odometry data through bluetooth to the GUI*/
void send_telemetry_data() {
}

/*TODO: Implement bluetooth receiving of op mode and start/stop states and reset*/
void receive_op_mode() {
}
/*TODO: Hardware reset for both boards*/
void reset_mcus() {
}
void send_controller_data(int32_t axisX, int32_t axisRX, int32_t axisRY) {
  if (main_serial_port.available()) {
    char tx_buffer[16];
    int max_buffer_size = 16;
    int max_value_length = 5;
    int separator_length = 2;

    int axisX_value = axisX;
    int axisRX_value = axisRX;
    int axisRY_value = axisRY;

    char buffer[max_buffer_size];

    int chars_written = snprintf(buffer, max_buffer_size, "%+d,%+d,%+d", axisX_value, axisRX_value, axisRY_value);

    if (chars_written < 0 || chars_written >= max_buffer_size) {
      /*TODO: Handle error, snprintf failed or buffer overflow occurred */
    } else {
      for (int i = chars_written; i < max_buffer_size - 1; i++) {
        buffer[i] = '\n';
      }
      buffer[max_buffer_size - 1] = '\0';
    }

    // Send the message over UART
    main_serial_port.printf("%s", buffer);
    Serial.printf("Sending over uart! %s", buffer);
  }
}

void tele_op_mode() {
  BP32.update();

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr myGamepad = myGamepads[i];

    if (myGamepad && myGamepad->isConnected()) {
      /*TODO: Remove serial print*/
      Serial.printf("LX: %d, LY: %d, RX: %d, RY: %d\n",
                    myGamepad->axisX(),    // (-511 - 512) left X Axis
                    myGamepad->axisY(),    // (-511 - 512) left Y axis
                    myGamepad->axisRX(),   // (-511 - 512) right X axis
                    myGamepad->axisRY());  // (-511 - 512) right Y axis
      /*TODO: Check if it works*/
      send_controller_data(myGamepad->axisX(), myGamepad->axisRX(), myGamepad->axisRY());
    }
  }
  delay(50);
}
/*TODO: Read the odometry data from the bluepill*/
void receive_odometry_data() {
  if (main_serial_port.available()) {
    // String rx_buffer = main_serial_port.readStringUntil('\n');
    // char rx_buffer[25];
    // main_serial_port.readBytes(rx_buffer, sizeof(rx_buffer));
    // String text = main_serial_port.read();
    // Serial.println(main_serial_port.read());
    //bt_serial_port.write(rx_buffer);

    static char message[26];
    static unsigned int message_pos = 0;
    //Read the next available byte in the serial receive buffer
    char inByte = main_serial_port.read();
    //Message coming in (check not terminating character) and guard for over message size
    if (inByte != '\n' && (message_pos < 26 - 1)) {
      //Add the incoming byte to our message
      message[message_pos] = inByte;
      message_pos++;
    }
    //Full message received...
    else {
      //Add null character to string
      message[message_pos] = '\0';
      //Print the message (or do other things)
      Serial.println(message);
      //Reset for the next message
      message_pos = 0;
    }
  }
}
void controller_setup() {
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
}

void setup() {
  controller_setup();
  Serial.begin(115200);

  // serial_bt.begin("RoadRunner");
  // Serial.println("The device started, now you can pair it with bluetooth!");

  main_serial_port.begin(115200, SERIAL_8N1, 16, 17);
  //bt_serial_port.begin("RoadRunner");
  //Serial.println("Device ready for pairing!");
}
char rx_buffer[26];
void loop() {
  /*TODO: Implement mode switch*/
  tele_op_mode();
  // if(serial_bt.available()){
  //   char incomingByte = serial_bt.read();  // Read the incoming byte
  //   Serial.print("Received: ");
  //   Serial.println(incomingByte);
  // }
}
