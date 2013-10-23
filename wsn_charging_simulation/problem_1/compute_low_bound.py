N = 20
E_c = 0.1855
E_init = 2.34 * 3600
e_UAV = 25.0 * 3600 * 2
e_charging = e_UAV * 45.0 / (75.0 + 45.0)
e_r = 0.3333
T = 604800

print N * (T * E_c - E_init) / (e_charging * e_r)