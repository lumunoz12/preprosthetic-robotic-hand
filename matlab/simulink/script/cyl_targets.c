#include "cyl_targets.h"

static float cil_poly(const float *c, int n, float x){
    float acc = c[0];
    int i; for(i=1;i<n;i++) acc = acc*x + c[i];
    return acc;
}

static float cil_clamp(float x){
    if(x<0.0f) return 0.0f; if(x>90.0f) return 90.0f; return x;
}

void cyl_compute_targets(float R, CylTargets *o){
    if (R < CYL_R_MIN) R = CYL_R_MIN;
    if (R > CYL_R_MAX) R = CYL_R_MAX;

    static const float c_i_mcp[] = { -2.898984074e-06f, 0.008861804274f, -1.286747432f, 98.64006164f };
    static const float c_i_pip[] = { 0.0001223962259f, -0.01006772395f, 0.5568318912f, 56.8750011f };
    static const float c_m_mcp[] = { 0.0001247500652f, -0.007430218261f, -0.7225614907f, 96.09147501f };
    static const float c_m_pip[] = { -6.893260582e-05f, 0.01207364186f, -0.07575112296f, 74.58327222f };
    static const float c_a_mcp[] = { -0.0002030341406f, 0.02141408856f, -1.495101761f, 105.4304506f };
    static const float c_a_pip[] = { 0.0003764856096f, -0.02764834118f, 1.012190358f, 56.04983437f };
    static const float c_q_mcp[] = { 0.0002756731321f, -0.02168449263f, -0.3964209967f, 100.9153103f };
    static const float c_q_pip[] = { 0.00178918229f, -0.1758913322f, 6.096914912f, -9.858879195f };
    static const float c_p_mcp[] = { 0.0009412836274f, -0.06860437146f, 0.1707329043f, 56.73236991f };
    static const float c_p_abd[] = { 6.419293372e-05f, 0.02421808545f, -1.172733766f, 72.73676188f };

    o->i_mcp = cil_clamp(cil_poly(c_i_mcp, 4, R));
    o->i_pip = cil_clamp(cil_poly(c_i_pip, 4, R));
    o->m_mcp = cil_clamp(cil_poly(c_m_mcp, 4, R));
    o->m_pip = cil_clamp(cil_poly(c_m_pip, 4, R));
    o->a_mcp = cil_clamp(cil_poly(c_a_mcp, 4, R));
    o->a_pip = cil_clamp(cil_poly(c_a_pip, 4, R));
    o->q_mcp = cil_clamp(cil_poly(c_q_mcp, 4, R));
    o->q_pip = cil_clamp(cil_poly(c_q_pip, 4, R));
    o->p_mcp = cil_clamp(cil_poly(c_p_mcp, 4, R));
    o->p_abd = cil_clamp(cil_poly(c_p_abd, 4, R));
}
