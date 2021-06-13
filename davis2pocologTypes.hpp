#ifndef davis2pocolog_TYPES_HPP
#define davis2pocolog_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace davis2pocolog {
    /** Color encodeing: first color positive event, seconds color negative event **/
    enum COLOR_ENCODING{BLUE_RED, GREEN_RED, BLUE_BLACK};
}

#endif

