#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include "libformat.h"

//Some fake empty variables to make gcc give up on stupid shit
const int __errno[0];
const char __aeabi_unwind_cpp_pr0[0];

//Very simplistic memset replacement
void * memset ( void * ptr, int value, size_t count ) {
	char *write = (char *)ptr;

	for (; count >0; --count ) {
		*write++ = value;
	}
	return ptr;
}


void * memcpy ( void * destination, const void * source, size_t count ) {
	char * read = (char*) source;
	char * write = (char *) destination;

	for (; count >0; --count ) {
		*write++ = *read++;
	}
	return destination;
}


void abort ( void ) {
	while ( 1 ) {
	}
}

void exit ( void ) {
	while ( 1 ) {
	}
}


int strcmp( const char* str1, const char* str2 ) {
	for ( ; ; str1++, str2++ ) {
		char c1 =*str1;
		char c2 = *str2;

		//Check for line endings on either string
		if ( !c1 ) {
			return c2 ? 1 : 0;
		} else if ( !c2 ) {
			return -1;
		}

		if ( c1 < c2 )
			return -1;
		if ( c1 > c2 )
			return 1;
	}
}

size_t strlen( const char* str ) {
	size_t count = 0;
	while( *str++ )
		++count;
	return count;
}

extern int strncasecmp( const char* str1, const char* str2, uint32_t max ) {
	for ( ; max; str1++, str2++, max-- ) {
		char c1 =*str1;
		char c2 = *str2;

		//Check for line endings on either string
		if ( !c1 ) {
			return c2 ? 1 : 0;
		} else if ( !c2 ) {
			return -1;
		}

		//Lowercase the case before comparing
		if ( c1 >= 'A' && c1 <= 'Z' )
			c1 += 32;
		if ( c2 >= 'A' && c2 <= 'Z' )
			c2 += 32;

		if ( c1 < c2 )
			return -1;
		if ( c1 > c2 )
			return 1;
	}
	//max triggered
	return 0;
}

char * strchr ( const char * s, int ch ) {
	if ( s ) {
		for( ; *s; s++ ) {
			if ( *s == ch )
				return (char*)s;
		}
	}
	return 0;
}

int32_t atoi( const char* input ) {
	int32_t result;

	makeInt( input, false, &result );
	return result;

}

uint32_t atou( const char* input ) {
	uint32_t result;

	makeInt( input, true, &result );
	return result;
}


////////////////////////////////////////////////////////////////////////////////
// String to Float Conversion
///////////////////////////////////////////////////////////////////////////////
// Simple and fast atof (ascii to float) function.
//
// - Executes about 5x faster than standard MSCRT library atof().
// - An attractive alternative if the number of calls is in the millions.
// - Assumes input is a proper integer, fraction, or scientific format.
// - Matches library atof() to 15 digits (except at extreme exponents).
// - Follows atof() precedent of essentially no error checking.
//
// 09-May-2009 Tom Van Baak (tvb) www.LeapSecond.com
//
#define white_space(c) ((c) == ' ' || (c) == '\t')
#define valid_digit(c) ((c) >= '0' && (c) <= '9')
float atof(const char *p)
{
    int frac = 0;
    double sign, value, scale;

    // Skip leading white space, if any.
    while (white_space(*p) ) {
        p += 1;
    }

    // Get sign, if any.
    sign = 1.0;
    if (*p == '-') {
        sign = -1.0;
        p += 1;

    } else if (*p == '+') {
        p += 1;
    }

    // Get digits before decimal point or exponent, if any.
    value = 0.0;
    while (valid_digit(*p)) {
        value = value * 10.0 + (*p - '0');
        p += 1;
    }

    // Get digits after decimal point, if any.
    if (*p == '.') {
        double pow10 = 10.0;
        p += 1;

        while (valid_digit(*p)) {
            value += (*p - '0') / pow10;
            pow10 *= 10.0;
            p += 1;
        }
    }

    // Handle exponent, if any.
    scale = 1.0;
    if ((*p == 'e') || (*p == 'E')) {
        unsigned int expon;
        p += 1;

        // Get sign of exponent, if any.
        frac = 0;
        if (*p == '-') {
            frac = 1;
            p += 1;

        } else if (*p == '+') {
            p += 1;
        }

        // Get digits of exponent, if any.
        expon = 0;
        while (valid_digit(*p)) {
            expon = expon * 10 + (*p - '0');
            p += 1;
        }
        if (expon > 308) expon = 308;

        // Calculate scaling factor.
        while (expon >= 50) { scale *= 1E50; expon -= 50; }
        while (expon >=  8) { scale *= 1E8;  expon -=  8; }
        while (expon >   0) { scale *= 10.0; expon -=  1; }
    }

    // Return signed and scaled floating point result.
    return sign * (frac ? (value / scale) : (value * scale));
}



#if 0
int __aeabi_idiv0 (int return_value) {
	abort();
	return 0;
}

long long __aeabi_ldiv0 (long long return_value) {
	abort();
	return 0;
}

#endif
