/*
 * libfunctions.h
 *
 *  Created on: 20 Jan 2013
 *      Author: Sjoerd
 */

#pragma once

#ifndef LIBFUNCTIONS_H_
#define LIBFUNCTIONS_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

//#define NULL 0

static inline int toupper( int c ) {
	if ( c >= 'a' && c <'z' ) {
		return c + ('A' - 'a');
	}
	return c;
}

extern void abort ( void );
extern void * memset ( void * ptr, int value, size_t count );
extern void * memcpy ( void * destination, const void * source, size_t num );

extern int strcmp( const char* str1, const char* str2 );
extern size_t strlen( const char* str );

char * strchr ( const char * input, int ch );

extern int strncasecmp( const char* str1, const char* str2, uint32_t max );

static inline int strcasecmp( const char* str1, const char* str2 ) {
	return strncasecmp( str1, str2, ~0 );
}

int32_t atoi( const char* input );

uint32_t atou( const char* input );

float atof( const char* input );

#endif /* LIBFUNCTIONS_H_ */
