void beep_buzzer()
{
  tone(BUZZER, 2500, 250);
  /*digitalWrite(BUZZER, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);*/
}
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
void turn_LED_on(int led_pin)
{
  digitalWrite(led_pin, HIGH);
}
void turn_LED_off(int led_pin)
{
  digitalWrite(led_pin, LOW);
}
void blink_led(int led_pin, int times)
{
  for(int i = 0; i<times; i++)
  {
    //digitalWrite(led_pin, !digitalRead(led_pin));
    turn_LED_on(led_pin);
    delay(250);
    turn_LED_off(led_pin);
    delay(250);
  }
}
