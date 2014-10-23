
#ifndef buffer_h
#define buffer_h

#include "Arduino.h"


// This method initializes the datastore of the buffer to a certain sizem the buffer should NOT be used before this call is made
void init(unsigned int buf_size);

// This method resets the buffer into an original state (with no data)
void clear();

// This releases resources for this buffer, after this has been called the buffer should NOT be used
void deAllocate();


// returns the byte at index in the buffer
byte peek(unsigned int index);

//
// Put methods, either a regular put in back or put in front
//
int putInFront(byte in);
int put(byte in);

//deletes data
byte get();
//deletes data
byte getFromBack();

int getInt();


byte* data;

unsigned int capacity;
unsigned int position;
unsigned int current_size;

#endif
