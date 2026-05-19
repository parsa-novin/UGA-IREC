#include "filters.h"

void Filter1P_init(Filter1P_t *f, float alpha)
{
    f->alpha = alpha;
    f->state = 0.0f;
    f->ready = false;
}

float Filter1P_update(Filter1P_t *f, float sample)
{
    if (!f->ready) {
        f->state = sample;
        f->ready = true;
    } else {
        f->state += f->alpha * (sample - f->state);
    }
    return f->state;
}
