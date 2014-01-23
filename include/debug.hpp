/*
 * File: debug.hpp
 * Author: Benedict R. Gaster
 * Desc: Debug helper code
 *
 * Copyright 2014 Benedict R. Gaster
 * License: See the file license.
 */
#pragma once

#if defined(__DEBUG__)
#include <stdio.h>
#define MSG(...) fprintf (stderr, __VA_ARGS__)
#else
#define MSG(...) 
#endif
