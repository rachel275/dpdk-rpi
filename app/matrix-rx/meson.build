# Minimal in-tree app recipe.
# Parent app/meson.build defines: sources, deps, includes, cflags, ldflags, etc.
# Here we just append our sources and (optionally) internal deps.

sources += files('matrix_rx.c')

# Internal DPDK libs this app uses (safe, small set).
deps += ['ethdev', 'mempool', 'mbuf', 'net']

# Optional compile define; parent already sets ALLOW_EXPERIMENTAL_API globally,
# so this line is not strictly needed, but harmless if you want it:
# cflags += ['-DALLOW_EXPERIMENTAL_API']

