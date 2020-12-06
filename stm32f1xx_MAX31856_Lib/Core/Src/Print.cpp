/*
 * Print.cpp
 *
 *  Created on: Aug 24, 2020
 *      Author: Macbook
 */

#include "Print.h"


using namespace std;

/**
 *  Constructor
 */
Print::Print()
{

}

/**
 *
 * @param size_buffer Número máximo de bytes que se utilizarán en el búfer.
 * @param format contiene una cadena de formato que sigue las mismas
 *        especificaciones que el formato en printf
 * @note  la funcion que utiliza para el formato es snprintf
 */
void Print::print( size_t size_buffer, const char * format, ... )
{
    char string[size_buffer];

    /* declara una lista que se puede usar para almacenar
     un número variable de argumentos. */
    va_list argp;

    /* va_start es una macro que acepta dos argumentos, un va_list y el nombre
     de la variable que precede directamente a la elipsis ("..."). */
    va_start( argp, format );

    if ( 0 < vsnprintf( string, size_buffer, format, argp ) )
    {
        write( (uint8_t*) string );
    }

    va_end( argp );
}

/**
 *
 * @param format  contiene una cadena de formato que sigue las mismas
 *        especificaciones que el formato en printf
 * @note  la funcion que utiliza para el formato es sprintf
 */
void Print::print( const char * format, ... )
{
    char string[200];

    /* declara una lista que se puede usar para almacenar
     un número variable de argumentos. */
    va_list argp;

    /* va_start es una macro que acepta dos argumentos, un va_list y el nombre
     de la variable que precede directamente a la elipsis ("..."). */
    va_start( argp, format );

    if ( 0 < vsprintf( string, format, argp ) ) // build string
    {
        write( (uint8_t*) string );  // funcion virtual
    }

    va_end( argp );

}

/**
 *
 * @param string_in texto de tipo string a transmitir
 */
void Print::print( string string_in )
{
    /* reserva memoria para el string */
    char text[string_in.length() + 1];

    /* convierte string a array de tipo char */
    strcpy( text, string_in.c_str() );

    write( (uint8_t*) text );

}

/**
 *
 * @param text_in cadena de texto a transmitir
 */
void Print::print( char * text_in )
{

    write( (uint8_t*) text_in );

}

/**
 *
 * @param text_in caracter o byte a transmitir
 */
void Print::print( uint8_t byte_in )
{

    write( &byte_in );

}

/**
 *
 * @param text_in dato de tipo uint8_t a transmitir
 */
void Print::print( uint8_t * bytes_in )
{

    write( bytes_in );

}

