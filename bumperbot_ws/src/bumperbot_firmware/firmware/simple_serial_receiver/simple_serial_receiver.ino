#define LED_PIN 13

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
}

// void loop() {
//   static String inputString = ""; // A String to hold incoming data
//   while (Serial.available()) {
//     char inChar = (char)Serial.read();
//     inputString += inChar;
//     // Check if the received character is the terminator
//     if (inChar == '\n') {
//       // Process the received string
//       int x = inputString.toInt();
//       if (x == 0) {
//         digitalWrite(LED_PIN, LOW);
//       } else {
//         digitalWrite(LED_PIN, HIGH);
//       }
//       // Clear the string for the next input
//       inputString = "";
//     }
//   }
//   delay(10); // Add a small delay
// }

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
  {
    int x = Serial.readString().toInt();
    if (x == 0)
    {
      digitalWrite(LED_PIN, LOW);
    }
    else{
      digitalWrite(LED_PIN, HIGH);
    }
  }

}