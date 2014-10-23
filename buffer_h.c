/*
 * buffer_h.c
 *
 *  Created on: Oct 23, 2014
 *      Author: lauril
 */

#include "Arduino.h"
#include "buffer_h.h"

void init(unsigned int buf_size){
	data = (byte*)malloc(sizeof(byte)*buf_size);
	capacity = buf_size;
	position = 0;
	current_size = 0;
}

int putInt(int in){
    byte *pointer = (byte *)&in;
    int ret1, ret2;
    ret1 = put(pointer[1]);
    if (ret1 != 0) {
    	return 1;
    }
    ret2 = put(pointer[0]);
    if(ret2 != 0){
    	//delete previous!
    	getFromBack(); //do't care about bytes
    	return 1;
    }
    return 0;
}

int getInt(){
	int ret;
    byte *pointer = (byte *)&ret;
	pointer[1] = get();
	pointer[0] = get();
	return ret;
}

byte peek(unsigned int index){
	byte b = data[(position+index)%capacity];
	return b;
}

int put(byte in){
	if(current_size < capacity){
		// save data byte at end of buffer
		data[(position+current_size) % capacity] = in;
		// increment the length
		current_size++;
		return 0;
	}
	// return failure
	return 1;
}

int putInFront(byte in){
	if(current_size < capacity){
		// save data byte at end of buffer
		if( position == 0 )
			position = capacity-1;
		else
			position = (position-1)%capacity;
		data[position] = in;
		// increment the length
		current_size++;
		return 0;
	}
	return 1;
}

byte get(){
	byte b = 0;

	if(current_size > 0){
		b = data[position];
		// move index down and decrement length
		position = (position+1)%capacity;
		current_size--;
	}

	return b;
}

byte getFromBack(){
	byte b = 0;
	if(current_size > 0){
		b = data[(position+current_size-1)%capacity];
		current_size--;
	}

	return b;
}

void clear(){
	position = 0;
	current_size = 0;
}

void deAllocate(){
	free(data);
}
