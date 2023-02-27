/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_SDE18S.h"

#if AP_RANGEFINDER_SDE18S_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// get a distance reading
bool AP_RangeFinder_SDE18S::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // read any aviable lines from sonar
    float sum = 0.0f;
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        if (decode(c)) {
            sum += _distance_m;
            count ++;
        }
    }

    // return false on failure
    if (count == 0) {
        return false;
    }

    // return average of all distance
    reading_m = sum / count;
    return true;
}

// get temperature reading in C. returns true on success and populates temp argument
bool AP_RangeFinder_SDE18S::get_temp(float &temp) const
{
    return false;
}



// add a single character to the buffer and attempt to decode
// returns true if a distance was successfully decoded
// distance should be pulled directly from _distance_m member
bool AP_RangeFinder_SDE18S::decode(char c)
{
    switch (c) {
    case '*': // end of a term 
    {
        if (_sentence_done) {
            return false;
        }

        // null terminate and decode latest term
        _term[_term_offset] = 0;
     
        // move onto next term
        _term_is_checksum = true;
        bool valid_sentence = decode_latest_term();
        _term_offset = 0;
        return valid_sentence;
    }
        FALLTHROUGH;
    
    case '@': // sentence begin
        _sentence_type = SONAR_UNKNOWN;
        _term_offset = 0;
        _term_is_checksum = false;
        _distance_m = -1.0f;
        _sentence_done = false;
        return false;
    }

    // ordinary characters are added to term
    if (_term_offset < sizeof(_term) - 1) {
        _term[_term_offset++] = c;
    }

    return false;
}


// decode the just-completed term
// returns true if new sentence has just passed checksum test and is validated
bool AP_RangeFinder_SDE18S::decode_latest_term()
{
    // handle the last term in a message
    if (_term_is_checksum) {
        _sentence_done = true;
        _sentence_type = SONAR_DPT;
        const char *term_value = &_term[22];
        _distance_m = strtof(term_value, NULL);
        return true;
    }
    return false;
}

#endif // AP_RANGEFINDER_SDE18S_ENABLED