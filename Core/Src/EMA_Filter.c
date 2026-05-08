#include "EMA_Filter.h"



void ema_init(ema_data_structure *f, float alpha) {
    f->alpha       = alpha;
    f->output      = 0.0f;
    f->initialized = false;
}

float ema_update(ema_data_structure *f, float input) {
    if (!f->initialized) {
        f->output      = input;  // Avoid startup transient
        f->initialized = true;
    } else {
        f->output = f->alpha * input + (1.0f - f->alpha) * f->output;
    }
    return f->output;
}
