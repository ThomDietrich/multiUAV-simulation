/*
 * Mission.cc
 *
 *  Created on: 15.03.2018
 *      Author: mremm
 */

#include "Mission.h"

Mission::Mission()
{
    
}

Mission::~Mission()
{
    dhis = first;
    LinkedElement<Command*>* next;
    while (nullptr != (next = dhis->next)) {
        delete dhis;
        dhis = next;
    }
}

void Mission::add(Command* command)
{
    if (finalized) throw cRuntimeError("Cannot add commands to finalized mission.");
    if (nullptr == dhis) {
        dhis = new LinkedElement<Command*>;
        dhis->index=0;
        first = dhis;
    }
    else {
        LinkedElement<Command*>* last = dhis;
        LinkedElement<Command*>* next = last->next;
        dhis = new LinkedElement<Command*>;
        dhis->index = last->index + 1;
        last->next = dhis;
        dhis->prev = last;
        dhis->element = command;
        if (nullptr != next) {
            dhis->next = next;
            next->prev = dhis;
        }
    }
    size+=1;
}

Mission& Mission::finalize()
{
    dhis->next = first;
    first->prev = dhis;
    finalized = true;
    return start();
}
