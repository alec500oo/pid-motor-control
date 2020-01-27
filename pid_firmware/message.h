/** message.h
 * This class is responsable for building and sending messages to the client PC. 
 * 
 * @author Alec Matthews <alec500oo\@gmail.com>
 */

#ifndef MESSAGE_H
#define MESSAGE_H

#include <Arduino.h>

class Message {

  Stream *ioStream = nullptr;

  unsigned char index = 0;
  char buf[15] = {0};

public:
  /* Can't default construct */
  Message() = delete;

  /**
   * Create a message class with a byte stream pointer
   * @param ioStream Stream pointer to get data from. This will most likely be a
   * serial stream
   */
  Message(Stream *ioStream) : ioStream(ioStream) { }

  /**
   * Get the next message from the serial stream.
   * @returns Nullptr if there are messages to process.
   */
  char* GetNextMessage();

  /** Erase stored message */
  void Clear();

};

#endif /* MESSAGE_H */
