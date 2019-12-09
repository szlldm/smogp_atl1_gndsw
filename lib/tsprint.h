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

#ifndef TSPRINTF_H
#define TSPRINTF_H

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>


void tsprintf_core(const char * format, va_list args);

void tsprintf(const char *format, ...);

void tsprintf_verbose(const char *format, ...);

void tsprintf_debug(const char *format, ...);

void tsprintf_set_verbose(bool verbose);

bool tsprintf_get_verbose(void);

void tsprintf_set_output(FILE * stream);

int tsprintf_init(bool use_spinlock, bool verbose, FILE * stream);



#endif
