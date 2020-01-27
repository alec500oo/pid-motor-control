#include "message.h"

char *Message::GetNextMessage() {
  if (ioStream->available()) {
    buf[index++] = ioStream->read();
    if (index >= 3 + buf[2]) {
      return buf;
    }
  }
  return nullptr;
}

void Message::Clear() {
  memset(buf, 0, 15);
  index = 0;
}
