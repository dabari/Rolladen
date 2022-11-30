float linADC(float avgSample){
    
  float countVoltage;
  float avgVoltage;
  float lsbCount;
  float lsbVoltage;
  
  float s[]={ 2295, 2323, 2345, 2368, 2390, // 09.50V - 09.90V
            2423, 2438, 2466, 2494, 2520, // 10.00V - 10.40V
            2552, 2574, 2606, 2630, 2655, // 10.50V - 10.90V
            2680, 2707, 2730, 2755, 2780, // 11.00V - 11.40V
            2806, 2830, 2855, 2880, 2911, // 11.50V - 11.90V
            2933, 2954, 2995, 3019, 3046, // 12.00V - 12.40V
            3078, 3105, 3131, 3157, 3185, // 12.50V - 12.90V
            3215, 3240, 3270, 3297, 3329, // 13.00V - 13.40V
            3353, 3390, 3418, 3452, 3477, // 13.50V - 13.90V
            3518, 3558, 3590, 3629, 3665, // 14.00V - 14.40V
            3705, 3735, 3775, 3815, 3855};// 14.50V - 14.90V

  if (avgSample <= s[54] & avgSample >= s[0]){

    for (int count; count < 55-1; count++){

      if (avgSample >= s[count] & avgSample <= s[count+1] ){

        lsbCount = avgSample - s[count];
        lsbVoltage = 0.1 / (s[count + 1] - s[count]);
        countVoltage = count * 0.1 + 9.5;
        avgVoltage = lsbCount * lsbVoltage + countVoltage;

      }      
    }
  } else {
    // Voltage too high or too low
    avgVoltage = 9999;
  }

  return avgVoltage;

}
