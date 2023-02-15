#pragma once

struct Power {
  Power();
  Power(unsigned char value);

  unsigned char value;


  bool isValid();

  operator int();
};
