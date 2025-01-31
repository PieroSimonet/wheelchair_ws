#ifndef ROSNEURO_FEEDBACK_ENGINE_HMM_BARS_CPP
#define ROSNEURO_FEEDBACK_ENGINE_HMM_BARS_CPP

#include "hmm_feedback/EngineHmmbars.h"

namespace rosneuro {

bars_engine::bars_engine() : bars() {}

// Basically this class create a engine that send the command to the wheelchair
// commander for what to do 

bars_engine::~bars_engine() {
    this->~bars();
}

}

#endif