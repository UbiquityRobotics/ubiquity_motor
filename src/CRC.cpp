/*
 * CRC.cpp
 *
 *  Created on: Apr 2, 2015
 *      Author: kurt
 */

#include <stdint.h>
#include <cstdio>
#include <stdlib.h>
#include <ros/console.h>
#include "CRC.h"

namespace ur {

CRC::CRC() {
  // TODO Auto-generated constructor stub

}

CRC::~CRC() {
  // TODO Auto-generated destructor stub
}

uint8_t CRC::checksum(const void *data_, size_t pos_, size_t len_) {
  uint8_t *data = (uint8_t *) data_;

  uint32_t r = 0;                    // do the math as an int
  uint8_t result = 0;               // return just the lsb byte

  for ( size_t i = pos_; i < pos_ + len_; i++ )
  {
    r = r + data[i];
  }

  r = r & 0xFF;

  result = (uint8_t) 0xFF - (uint8_t) r;
  return result;
}

} /* namespace ur */
