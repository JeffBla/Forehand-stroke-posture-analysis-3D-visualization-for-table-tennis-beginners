#ifndef TESTBED_DEBUG_H
#define TESTBED_DEBUG_H

#ifndef __DEBUG_H__
#define __DEBUG_H__

/**
 This Debug Header is used improve debugging.
 @see https://github.com/ttocsneb/cs235resources
 @code
 #define DEBUG
 #include "debug.h"
 //Insert rest of code here
 @endcode
 There are a few macros that are enabled by DEBUG and disabled without it.
 */

#ifdef DEBUG
#include <string>
#include <iostream>
#include <iomanip>

/**
 DEBUG_PRINT(x)
 Debug print to cout withought an endline
 @param x anything compatible with cout, even expressions work.
 @code
 DEBUG_PRINT("foo" << bar);
 @endcode
 */
#define DEBUG_PRINT(x) std::cout << x
/**
 DEBUG_PRINTLN(x)
 Print to cout with an endline.
 @param x anything compatible with cout.
 */
#define DEBUG_PRINTLN(x) std::cout << x << std::endl

void debugPrintBar(std::string input);
/**
 DEBUG_PRINT_BAR(x)
 Print a Bar of `=` with centered text inside
 @param x a string of what to be displayed
 @code
 DEBUG_PRINT_BAR("HELLO WORLD");
 @endcode
 I redefined the function as a macro because I like the way visual studio highlights them,
 also I wanted to keep it standardized.
 */
#define DEBUG_PRINT_BAR(x) debugPrintBar(x)

void debugPrintBar(std::string input) {
    const int width = 119;
    int size = input.length();
    std::cout << std::setw(width / 2 - size / 2) << std::setfill('=') << "="
              << std::setw(width / 2 + size / 2) << std::left << input << std::endl;
}

#else
// When DEBUG is not defined, we still need to define the functions from above, so the
// precompiler knows to remove it during compile time.
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT_BAR(x)

#endif

#endif

/**
 VS_MEM_CHECK
 The Memory leak check that was provided to us.
 @note The check is not disabled when DEBUG is not defined.
 @code
 VS_MEM_CHECK;
 @endcode
 */

#ifdef _MSC_VER
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#define VS_MEM_CHECK _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF)
#else
#define VS_MEM_CHECK
#endif

#endif //TESTBED_DEBUG_H
