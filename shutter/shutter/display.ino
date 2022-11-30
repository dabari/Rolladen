

#ifdef OLED_Display    // ######################

void MoveModeOLEDwrite(String text, float val1, int val2)
{
      display.clearDisplay();
      display.setTextSize(2);             
      display.setTextColor(SSD1306_WHITE);       
      display.setCursor(0, 0);            
      display.println(text);
      display.print("[");
      display.print(int((val1 / val2) * 100));
      display.print("%]");
      display.display();  
}

void CalModeOLEDwrite(String txt1, String txt2)
{
      display.clearDisplay();
      display.setTextSize(2);             // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);        // Draw white text
      display.setCursor(0, 0);            // Start at top-left corner
      display.println(txt1);
      display.print(txt2);
      display.display();
}

void IPAddrOLEDwrite(String myIP, String txt2)
{
      display.clearDisplay();
      display.setTextSize(1);             // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);        // Draw white text
      display.setCursor(0, 0);            // Start at top-left corner
      display.println(myIP);
      display.print(txt2);
      display.display();
}

void LowVoltageOLEDwrite(String text, String val1)
{
      display.clearDisplay();
      display.setTextSize(2);             
      display.setTextColor(SSD1306_WHITE);       
      display.setCursor(0, 0);            
      display.println(text);
      display.print("[");
      display.print(val1);
      display.print("V]");
      display.display();  
}
#endif                 // ######################

