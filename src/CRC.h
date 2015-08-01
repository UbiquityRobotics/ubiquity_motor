/*
 * CRC.h
 *
 *  Created on: Apr 2, 2015
 *      Author: kurt
 */

#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>
#include <stdlib.h>


namespace ur {

class CRC {
public:
  CRC();
  virtual ~CRC();

  /*
   * Amrit needs the CRC to not include the delimiter byte.  So this
   * checksum takes both a position and a length.
   *    parameters:
   *          data_ - The array of bytes on which to perform the checksum
   *          pos_  - The starting byte on which to perform the checksum
   *          len_  - The number of bytes on which to perform the checksum
   *
   *    Return value: the one byte checksum based on Amrit's Serial Comm
   *    interface to the motor controller (version #3)
   *
   */
uint8_t checksum(const void *data_, size_t pos_, size_t len_);
};

} /* namespace ur */

#endif /* CRC_H_ */
