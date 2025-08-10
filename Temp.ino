// Blink the built-in LED on an Arduino Mega
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize pin as output
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // Turn LED on
  delay(1000);                      // Wait 1 second
  digitalWrite(LED_BUILTIN, LOW);   // Turn LED off whys this red
  delay(1000);    
                    // Wait 1 second
  digitalWrite(LED_BUILTIN, HIGH);   // Turn LED off whys this red
  delay(1000);  
  // bananas                c
}
