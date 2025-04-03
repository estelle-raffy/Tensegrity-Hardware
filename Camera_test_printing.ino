// simple code for receiveing python coordinates/error and printing back in python

// error -------------------------------------------------------------------

float error_distance = 0.0;

void setup() {
  Serial.begin(9600);  // Start serial communication
}

void loop() {
  receiveErrorFromPython();  // Read error value from Python
}

void receiveErrorFromPython() {
  if (Serial.available() > 0) {
    // Read the incoming data from Python
    String input = Serial.readStringUntil('\n');  // Read until newline
    input.trim();  // Remove any extra spaces or newline characters

    // Convert the received string to a float (error distance)
    float received_error = input.toFloat();

    // If valid (not NaN), store and print it
    if (!isnan(received_error)) {
      error_distance = received_error;
      Serial.println(error_distance);
    } else {
      Serial.println("[Arduino] Invalid data received.");
    }
  }
}

// coordinates ----------------------------------------------------------------
//float current_X = 0.0;
//float current_Y = 0.0;
//
//void setup() {
//
//  Serial.begin(9600);
//
//}
//
//void loop() {
//
//  parseCoordinates();
//
//}
//
//void parseCoordinates() {
//  if (Serial.available() > 0) {  // use Serial1 (pins 19, 18) for new pathways with Python
//    // Read the incoming data from the serial buffer
//    String input = Serial.readStringUntil('\n');  // Read until newline, may block code if python doesn't send updates regularly
//    int commaIndex = input.indexOf(',');  // Find the comma separating X and Y
//
//    // Extract the X and Y values from the string
//    if (commaIndex > 0) {
//      String xStr = input.substring(0, commaIndex);  // Extract X part
//      String yStr = input.substring(commaIndex + 1);  // Extract Y part
//
//      // Convert the strings to floats and store them in X and Y
//      current_X = xStr.toFloat();
//      current_Y = yStr.toFloat();
//      Serial.print(current_X);
//      Serial.print(",");
//      Serial.println(current_Y);
//
//    }
//  }
//}
