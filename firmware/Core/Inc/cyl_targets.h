#ifndef CYL_TARGETS_H
#define CYL_TARGETS_H

/* Auto-generado por generar_cil_polinomios.m */

#define CYL_R_MIN  15.0000f
#define CYL_R_MAX  45.0000f

typedef struct {
    float i_mcp, i_pip;
    float m_mcp, m_pip;
    float a_mcp, a_pip;
    float q_mcp, q_pip;
    float p_mcp, p_abd;
} CylTargets;

void cyl_compute_targets(float R, CylTargets *o);

#endif
