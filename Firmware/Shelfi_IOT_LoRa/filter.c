// out of storage … If you run out of space, you can't save to Drive or back up Google Photos.
#define Ntap 31
 const float FIRCoef[Ntap] = {

    //    100 1 2 31 butter  seems good
    0.01919203410827458400,
    0.02064571450085526300,
    0.02217328101588215600,
    0.02377639501600426300,
    0.02545660790719416300,
    0.02721534706922783600,
    0.02905390115540181300,
    0.03097340477464165600,
    0.03297482256732658900,
    0.03505893269532646400,
    0.03722630976536391000,
    0.03947730721445933500,
    0.04181203918528315200,
    0.04423036192935453400,
    0.04673185477553580500,
    0.04800337263973703000,
    0.04673185477553580500,
    0.04423036192935453400,
    0.04181203918528315200,
    0.03947730721445933500,
    0.03722630976536391000,
    0.03505893269532646400,
    0.03297482256732658900,
    0.03097340477464165600,
    0.02905390115540181300,
    0.02721534706922783600,
    0.02545660790719416300,
    0.02377639501600426300,
    0.02217328101588215600,
    0.02064571450085526300,
    0.01919203410827458400


  };
float fir1(float NewSample) {
 

  static float x[Ntap]; //input samples
  float y = 0;          //output sample
  int n;

  //shift the old samples
  for (n = Ntap - 1; n > 0; n--)
    x[n] = x[n - 1];

  //Calculate the new output
  x[0] = NewSample;
  for (n = 0; n < Ntap; n++)
    y += FIRCoef[n] * x[n];

  return y;
}

 
float fir2(float NewSample) {
  
  static float x[Ntap]; //input samples
  float y = 0;          //output sample
  int n;

  //shift the old samples
  for (n = Ntap - 1; n > 0; n--)
    x[n] = x[n - 1];

  //Calculate the new output
  x[0] = NewSample;
  for (n = 0; n < Ntap; n++)
    y += FIRCoef[n] * x[n];

  return y;
}

float fir3(float NewSample) {
  
  static float x[Ntap]; //input samples
  float y = 0;          //output sample
  int n;

  //shift the old samples
  for (n = Ntap - 1; n > 0; n--)
    x[n] = x[n - 1];

  //Calculate the new output
  x[0] = NewSample;
  for (n = 0; n < Ntap; n++)
    y += FIRCoef[n] * x[n];

  return y;
}

float fir4(float NewSample) {
  
  static float x[Ntap]; //input samples
  float y = 0;          //output sample
  int n;

  //shift the old samples
  for (n = Ntap - 1; n > 0; n--)
    x[n] = x[n - 1];

  //Calculate the new output
  x[0] = NewSample;
  for (n = 0; n < Ntap; n++)
    y += FIRCoef[n] * x[n];

  return y;
}

 