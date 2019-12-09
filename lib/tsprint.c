/*
 * Thread safe printf
 * Copyright (C) 2017 szlldm
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "tsprint.h"

static pthread_mutex_t print_message_mutex = PTHREAD_MUTEX_INITIALIZER;

static pthread_spinlock_t print_message_spinlock;

static FILE * print_output_stream = NULL;
static bool print_message_use_spinlock = false;
static bool print_message_verbose = false;

void tsprintf_core(const char * format, va_list args)
{
	if (print_message_use_spinlock) {
		pthread_spin_lock( &print_message_spinlock );
	} else {
		pthread_mutex_lock( &print_message_mutex );
	}

	if (print_output_stream == NULL) {
		print_output_stream = stderr;
	}
	if (format != NULL) {
		vfprintf(print_output_stream, format, args);
		fflush(print_output_stream);
	}
	if (print_message_use_spinlock) {
		pthread_spin_unlock( &print_message_spinlock );
	} else {
		pthread_mutex_unlock( &print_message_mutex );
	}
}

void tsprintf(const char *format, ...)
{
	if (format == NULL) return;

	va_list args;
	va_start(args, format);
	tsprintf_core(format, args);
	va_end(args);
}

void tsprintf_verbose(const char *format, ...)
{
	if (!print_message_verbose) return;

	if (format == NULL) return;

	va_list args;
	va_start(args, format);
	tsprintf_core(format, args);
	va_end(args);

}

void tsprintf_debug(const char *format, ...)
{
  #ifdef DEBUG
	if (format == NULL) return;

	va_list args;
	va_start(args, format);
	tsprintf_core(format, args);
	va_end(args);
  #endif
}

void tsprintf_set_verbose(bool verbose)
{
	print_message_verbose = verbose;
}

bool tsprintf_get_verbose(void)
{
	return print_message_verbose;
}

void tsprintf_set_output(FILE * stream)
{
	print_output_stream = stream;
}

int tsprintf_init(bool use_spinlock, bool verbose, FILE * stream)
{
	if (stream == NULL) return -1;

	print_message_use_spinlock = use_spinlock;
	if (pthread_spin_init(&print_message_spinlock, PTHREAD_PROCESS_SHARED ) != 0) return -1;
	print_message_verbose = verbose;
	print_output_stream = stream;

	return 0;
}








