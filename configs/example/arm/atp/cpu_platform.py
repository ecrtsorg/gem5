# Copyright (c) 2016-2017,2019-2021,2023 Arm Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

import os
import m5
from m5.util import addToPath
from m5.objects import *
from m5.options import *
import argparse

m5.util.addToPath("..")
m5.util.addToPath("../../..")

from common import SysPaths
from common import MemConfig
from common import ObjectList
from common.cores.arm import O3_ARM_v7a

import devices
import workloads


class RTComputeSystem(devices.BaseSimpleSystem):
    def __init__(self, mem_size, platform, **kwargs):
        super().__init__(mem_size, platform, **kwargs)
        # Coherent interconnect
        self.cibus = SystemXBar()
        # Non-coherent interconnect
        self.iobus = IOXBar()
        # Memory bus for multi-port Memory Controllers
        self.membus = IOXBar()

        self.iobridge = Bridge(delay="50ns")

    def connect(self):
        # Memory access: CI | NCI -> Memory bus
        self.cibus.mem_side_ports = self.membus.cpu_side_ports
        self.iobus.mem_side_ports = self.membus.cpu_side_ports

        # I/O peripherals, non-coherent devices: Devices -> NCI
        mem_ports = []
        self.realview.attachOnChipIO(
            self.iobus, self.iobridge, mem_ports=mem_ports
        )
        self.realview.attachIO(self.iobus)

        # Memory access: Memory bus -> Other memories
        for mem_port in mem_ports:
            self.membus.mem_side_ports = mem_port

        # I/O programming: CI -> Bridge -> NCI
        self.cibus.mem_side_ports = self.iobridge.cpu_side_port
        self.iobridge.mem_side_port = self.iobus.cpu_side_ports
        # self.iobridge.ranges += [AddrRange(0x20000000, 0x2F000000)]

        self.system_port = self.cibus.cpu_side_ports

    def addCaches(self):
        for cpu_cluster in self._clusters:
            if cpu_cluster.memoryMode() == "timing":
                cpu_cluster.addL1()
                cpu_cluster.addL2(cpu_cluster.clk_domain)
            cpu_cluster.connectMemSide(self.cibus)


class AtpPlatform(VExpress_GEM5_Foundation):
    def __init__(self, atp_files, **kwargs):
        super().__init__(**kwargs)
        self.atp_adaptor = ProfileGen(config_files=atp_files)

    def _on_chip_devices(self):
        return super()._on_chip_devices()

    def attachOnChipIO(self, bus, bridge=None, dma_ports=None, mem_ports=None):
        super().attachOnChipIO(bus, bridge, dma_ports, mem_ports)

        for device in self.atp_adaptor.config_files:
            self.atp_adaptor.port = bus.cpu_side_ports


# Pre-defined CPU configurations. Each tuple must be ordered as : (cpu_class,
# l1_icache_class, l1_dcache_class, walk_cache_class, l2_Cache_class). Any of
# the cache class may be 'None' if the particular cache is not present.
cpu_types = {
    "minor": (MinorCPU, devices.L1I, devices.L1D, devices.L2),
    "o3": (
        O3_ARM_v7a.O3_ARM_v7a_3,
        O3_ARM_v7a.O3_ARM_v7a_ICache,
        O3_ARM_v7a.O3_ARM_v7a_DCache,
        O3_ARM_v7a.O3_ARM_v7aL2,
    ),
}


def get_atp_files(atp_files_path):
    atp_files = set()
    for path in atp_files_path:
        for dname, _, fnames in os.walk(path):
            atp_files.update([os.path.join(dname, fname) for fname in fnames])
    return list(atp_files)


def create_cow_image(name):
    """Helper function to create a Copy-on-Write disk image"""
    image = CowDiskImage()
    image.child.image_file = name
    return image


def create(args):
    """Create and configure the system object."""

    if args.readfile and not os.path.isfile(args.readfile):
        print("Error: Bootscript %s does not exist" % args.readfile)
        sys.exit(1)

    object_file = args.kernel if args.kernel else ""
    atp_files = get_atp_files(args.atp_files_path)

    system = RTComputeSystem(
        args.mem_size,
        platform=AtpPlatform(atp_files=atp_files),
        mem_mode="timing",
        readfile=args.readfile,
    )

    MemConfig.config_mem(args, system)

    if args.disk_image:
        # Create a VirtIO block device for the system's boot
        # disk. Attach the disk image using gem5's Copy-on-Write
        # functionality to avoid writing changes to the stored copy of
        # the disk image.
        system.realview.vio[0].vio = VirtIOBlock(
            image=create_cow_image(args.disk_image)
        )

    # Wire up the system's memory system
    system.connect()

    # Add CPU clusters to the system
    system.cpu_cluster = [
        devices.CpuCluster(
            system, args.num_cores, args.cpu_freq, "1.0V", *cpu_types[args.cpu]
        )
    ]

    # Create a cache hierarchy for the cluster. We are assuming that
    # clusters have core-private L1 caches and an L2 that's shared
    # within the cluster.
    system.addCaches()

    workload_class = workloads.workload_list.get(args.workload)
    system.workload = workload_class(object_file, system)

    return system


def run(args):
    cptdir = m5.options.outdir
    if args.checkpoint:
        print("Checkpoint directory: %s" % cptdir)

    while True:
        event = m5.simulate()
        exit_msg = event.getCause()
        if exit_msg == "checkpoint":
            print("Dropping checkpoint at tick %d" % m5.curTick())
            cpt_dir = os.path.join(m5.options.outdir, "cpt.%d" % m5.curTick())
            m5.checkpoint(os.path.join(cpt_dir))
            print("Checkpoint done.")
        else:
            print(exit_msg, " @ ", m5.curTick())
            break

    sys.exit(event.getCode())


def main():
    parser = argparse.ArgumentParser(epilog=__doc__)

    parser.add_argument(
        "--atp-files-path",
        type=str,
        nargs="*",
        default=[],
        help="Paths to ATP Files to load",
    )
    parser.add_argument(
        "--kernel", type=str, default=None, help="Binary to run"
    )
    parser.add_argument(
        "--workload",
        type=str,
        default="ArmBaremetal",
        choices=workloads.workload_list.get_names(),
        help="Workload type",
    )
    parser.add_argument(
        "--disk-image", type=str, default=None, help="Disk to instantiate"
    )
    parser.add_argument(
        "--readfile",
        type=str,
        default="",
        help="File to return with the m5 readfile command",
    )
    parser.add_argument(
        "--cpu",
        type=str,
        choices=list(cpu_types.keys()),
        default="o3",
        help="CPU model to use",
    )
    parser.add_argument("--cpu-freq", type=str, default="4GHz")
    parser.add_argument(
        "--num-cores", type=int, default=1, help="Number of CPU cores"
    )
    parser.add_argument(
        "--machine-type",
        type=str,
        choices=ObjectList.platform_list.get_names(),
        default="VExpress_GEM5_V2",
        help="Hardware platform class",
    )
    parser.add_argument(
        "--mem-type",
        default="DDR3_1600_8x8",
        choices=ObjectList.mem_list.get_names(),
        help="type of memory to use",
    )
    parser.add_argument(
        "--mem-channels", type=int, default=4, help="number of memory channels"
    )
    parser.add_argument(
        "--mem-channels-intlv",
        type=int,
        default=256,
        help="Memory channels interleave",
    )
    parser.add_argument(
        "--mem-ranks",
        type=int,
        default=None,
        help="number of memory ranks per channel",
    )
    parser.add_argument(
        "--mem-size",
        action="store",
        type=str,
        default="2GB",
        help="Specify the physical memory size",
    )
    parser.add_argument("--checkpoint", action="store_true")
    parser.add_argument("--restore", type=str, default=None)
    parser.add_argument(
        "-P",
        "--param",
        action="append",
        default=[],
        help="Set a SimObject parameter relative to the root node. "
        "An extended Python multi range slicing syntax can be used "
        "for arrays. For example: "
        "'system.cpu[0,1,3:8:2].max_insts_all_threads = 42' "
        "sets max_insts_all_threads for cpus 0, 1, 3, 5 and 7 "
        "Direct parameters of the root object are not accessible, "
        "only parameters of its children.",
    )

    args = parser.parse_args()

    root = Root(full_system=True)
    root.system = create(args)

    root.apply_config(args.param)

    if args.restore is not None:
        m5.instantiate(args.restore)
    else:
        m5.instantiate()

    run(args)


if __name__ == "__m5_main__":
    main()
