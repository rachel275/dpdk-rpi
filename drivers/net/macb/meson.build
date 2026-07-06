# drivers/net/macb/meson.build

allow_experimental_apis = true

# Tell the top-level build which DPDK libs we need
deps += ['ethdev', 'kvargs', 'bus_vdev', 'mempool', 'net']

# Our sources
sources = files(
  'macb_ethdev.c',
  'macb_rxtx.c',
  'macb_uio.c',
  'macb_phy.c',
  'macb_dma.c',
  'macb_ring.c',
  'macb_ctrl.c',
)

# Optional flags/includes
cflags += ['-Wno-unused-parameter', '-Wno-missing-field-initializers']
# includes += include_directories('.')

# Optional headers to install with the PMD
install_headers = files('macb_hw.h')

name = 'rte_net_macb'
