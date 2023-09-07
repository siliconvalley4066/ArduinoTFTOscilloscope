byte interleave, ilv;

void i_scaleDataArray() {
  byte *pdata, *qdata;
  int *idata, prev;
  long a;

  if (interleave == 0) prev = ilv - 1;
  else prev = interleave - 1;
  idata = (int *) data[sample+1];
  pdata = data[sample] + interleave;
  if (sample == 2) {
    qdata = data[0] + prev;
  } else {
    qdata = data[2] + prev;
  }
  for (int i = 0; i < SAMPLES/ilv; i++) {
    a = ((*idata++ + ch0_off) * VREF[range0] + 512) >> 10;
    if (a > LCD_YMAX) a = LCD_YMAX;
    else if (a < 0) a = 0;
    if (ch0_mode == MODE_INV)
      a = LCD_YMAX - a;
    *pdata = (byte) a;
    *(pdata + prev - interleave) = *qdata;
    pdata += ilv;
    qdata += ilv;
  }
}
