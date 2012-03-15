// this module is used to keep track of the global game timeout
// setup starts the timer
// globalTimeout returns true once time is up

#pragma once

// setup, call once at start
void globalTimeoutSetup();

// returns true if game time is up
char globalTimeout();
