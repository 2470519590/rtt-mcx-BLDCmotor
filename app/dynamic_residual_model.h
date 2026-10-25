#ifndef DYNAMIC_RESIDUAL_MODEL_H
#define DYNAMIC_RESIDUAL_MODEL_H
/* Generated PI-lookahead candidate; keep separate from legacy nn_model.h. */
#define DYNAMIC_INPUTS 3U
#define DYNAMIC_HIDDEN 8U
static const float dynamic_w1[8][3] = {
    {1.06578776f, 0.290224122f, -1.44871305f},
    {0.772534414f, 1.06646188f, 0.0852134674f},
    {0.0915526739f, 0.888946138f, -1.84035113f},
    {0.312701007f, 1.92088607f, -1.68817631f},
    {-1.79555892f, -1.66200595f, -2.01831716f},
    {-1.66322691f, 0.922538948f, -3.10614122f},
    {0.504919282f, 0.241814262f, -0.689987113f},
    {-0.0196627664f, 0.939311638f, -1.14967809f},
};
static const float dynamic_b1[8] = {0.363189876f, -0.313463658f, 0.0565133654f, -0.214025065f, -0.563862026f, 0.458165944f, 0.175064266f, -0.0462210141f};
static const float dynamic_w2[8] = {0.00107605225f, -0.00405193347f, 0.00365103735f, -0.00104812998f, 0.00208039695f, -0.00390804913f, 0.0027949933f, -0.0023666189f};
static const float dynamic_b2 = 0.000480676332f;
static const float dynamic_linear[4] = {0.00529045472f, -0.00647952547f, 0.00083507027f, 5.96671271e-06f};
#endif
