// |***************************************************************|
// |* (c) Copyright 2024                                           |
// |* Balloon Payload Program, University of Maryland              |
// |* College Park, MD 20742                                       |
// |* https://bpp.umd.edu                                          |
// |***************************************************************|

// ---- BOBOT-1 SHOULDER SERVO VALIDATION CODE ---- //
// Counts Per revolution (according to data sheet: 48, since 12 ppr))
// Counts per revolution @ output: 48*200 = 57600!

// GLOBAL VARIABLES:
#define CHANNEL_A 2 // Interruptable pins
#define CHANNEL_B 3 // Interruptable pins
#define IN1 48
#define IN2 50

// Important stuff for keeping track of quadrature state
volatile static int32_t quad_count, quad_mailbox; // The encoder counts, and
volatile static uint8_t quad_state; // current state of the system
volatile static uint8_t quad_flag; // a flag for

// Following information from here: https://forum.digikey.com/t/design-a-robust-quadrature-encoder-isr-based-program-for-arduino-motor-control/39913
void setup() {
  // First, set the pin modes
  pinMode(CHANNEL_A, INPUT);
  pinMode(CHANNEL_B, INPUT);

  // Now, attach them to interrupts
  attachInterrupt(digitalPinToInterrupt(CHANNEL_A), QUAD_interrupt_func, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_B), QUAD_interrupt_func, CHANGE);

  // Initialize the quadrature stuff (and things)
  init_QUAD();
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

}

void init_QUAD()
{
   uint8_t A = digitalRead(CHANNEL_A);
   uint8_t B = digitalRead(CHANNEL_B);
   uint8_t BA = (B << 1) + A;
  // uint8_t BA = VPORTD.IN & 0x03;  // Use direct port access to save time <- to implement on LattePanda?
  quad_state = BA; 
  quad_count = 0;
  quad_flag = 1;
}

void QUAD_interrupt_func()
{
  uint8_t A, B, BA;
  digitalWrite(LED_BUILTIN, 1); // Turn on the LED to show we've interrupted

  // If we are in an error state, just return
  if(quad_state == 0xFF){return;}

  // Get the quadrature values
  A = digitalRead(CHANNEL_A);
  B = digitalRead(CHANNEL_B);
  BA = (B << 1) + A;

  // Use a fininte state machine to switch between states
  // For a description of the state machine see https://forum.digikey.com/t/quadrature-encoder-system-integration/39576
  switch (quad_state) {                     
    case 0b00:
      if (BA == 0b00) 
      {
        // pass
      } 
      else if (BA == 0b01) 
      {
        quad_state = 0b01;
        quad_count+=1; // Add counts
      } 
      else if (BA == 0b10) 
      {
        quad_state = 0b10;
        quad_count-=1; // Subtract counts
      } 
      else {quad_state = 0xFF;} // again, if in error state, don't do anything
      break;

    case 0b01:
      if (BA == 0b01) 
      {
        // pass
      } 
      else if (BA == 0b11) 
      {
        quad_state = 0b11;
        quad_count+=1; // add counts
      } else if (BA == 0b00) 
      {
        quad_state = 0b00;
        quad_count-=1; // subtract counts
      } 
      else {quad_state = 0xFF;}
      break;
    
    // pretty much the same as above, just see those comments you goober - romeo
    case 0b11:
      if (BA == 0b11) 
      {
        // pass
      } 
      else if (BA == 0b10) 
      {
        quad_state = 0b10;
        quad_count+=1;
      } 
      else if (BA == 0b01) 
      {
        quad_state = 0b01;
        quad_count-=1;
      } 
      else {quad_state = 0xFF;}
      break;

    case 0b10:
      if (BA == 0b10) 
      {
        // pass
      } 
      else if(BA == 0b00) 
      {
        quad_state = 0b00;
        quad_count+=1;
      } 
      else if (BA == 0b11) 
      {
        quad_state = 0b11;
        quad_count-=1;
      } 
      else {quad_state = 0xFF;}
      break;
  }

  // do something? 
  if (quad_flag) 
  {
    quad_mailbox = quad_count;
    quad_flag = 0;
  }

  digitalWrite(LED_BUILTIN, 0); // Turn the LED off
}

int32_t get_curr_quad(int timeout_ms)
{
  // I won't sugar coat it folks, this is pretty much copy and pasted from the website

  int32_t last_known_position; // the last know position
  int32_t startMillis = millis(); // a milisecond value

  // Retrieve the latest data if no request is pending
  if(!quad_flag) 
  {                         
    last_known_position = quad_mailbox;
  }

  // Request new data. This line is redundant if the flag is already set from a previous iteration.
  quad_flag = 1;                            

  while(quad_flag) 
  {
    if(millis() - startMillis > timeout_ms) 
    {
      return last_known_position;
    }
  }

  return quad_mailbox;
}

void loop()
{
  while (quad_state == 0xFF) 
  {
    Serial.println("Halt due to quadrature sensor positioning error.");
    delay(1000);
  }

  int32_t encoder_counts_pos = get_curr_quad(10);
  Serial.println(encoder_counts_pos*2*PI/(9600)*180/PI);
}

