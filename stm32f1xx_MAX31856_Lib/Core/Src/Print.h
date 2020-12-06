/*
 * Print.h
 *
 *  Created on: Aug 24, 2020
 *      Author: Macbook
 */

#ifndef SRC_PRINT_H_
#define SRC_PRINT_H_



#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "string"

using namespace std;

class Print
{

public:

    Print();
    virtual void write(uint8_t *text)=0;

    void print( size_t size_buffer, const char * format, ... );
    void print( const char * format, ... );
    void print( string string_in );
    void print( char * text_in );
    void print( uint8_t byte_in );
    void print( uint8_t * bytes_in );

};

#endif /* SRC_PRINT_H_ */
