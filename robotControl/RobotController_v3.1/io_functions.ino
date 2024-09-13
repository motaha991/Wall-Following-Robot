
bool button_is_pressed(int but)
{
  if(digitalRead(but) == 0) // Button is pressed
  {
    delay(200); // A delay of 200 ms to counter bouncing problems.
    if(digitalRead(but) == 0) // Button is pressed
    return(true);
  }
  if(digitalRead(but) != 0) // Button is not pressed
  {
    delay(200); // A delay of 200 ms to counter bouncing problems.
    if(digitalRead(but) != 0) // Button is not pressed
    return(false);
  }  
  return(true);   // button is pressed
}

void blink_led(int times)
{
  for(int i = 0; i<times; i++)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
}
