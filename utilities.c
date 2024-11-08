#include "utilities.h"

static int state = 0; // current state defaults to idle

int get_state(void) {
    return state;
};

void set_state(int new_state){
    state = new_state;
}

