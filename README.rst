Qemu fork to interface with SimulIDE.

Build:

.. code-block:: shell

  ./configure --target-list=xtensa-softmmu --extra-cflags=-fPIC --disable-attr --disable-auth-pam --disable-avx2 --disable-avx512bw --disable-blkio --disable-bochs --disable-bpf --disable-brlapi --disable-bzip2 --disable-canokey --disable-cap-ng --disable-capstone --disable-cloop --disable-cocoa --disable-colo-proxy --disable-coreaudio --disable-crypto-afalg --disable-curl --disable-curses --disable-dbus-display --disable-dmg --disable-docs --disable-dsound --disable-fuse --disable-fuse-lseek --disable-gcrypt --disable-gettext --disable-gio --disable-glusterfs --disable-gnutls --disable-gtk --disable-gtk-clipboard --disable-guest-agent --disable-guest-agent-msi --disable-hvf --disable-iconv --disable-jack --disable-keyring --disable-kvm --disable-l2tpv3 --disable-libdaxctl --disable-libdw --disable-libiscsi --disable-libkeyutils --disable-libnfs --disable-libpmem --disable-libssh --disable-libudev --disable-libusb --disable-libvduse --disable-linux-aio --disable-linux-io-uring --disable-lzfse --disable-lzo --disable-malloc-trim --disable-membarrier --disable-modules --disable-mpath --disable-multiprocess --disable-netmap --disable-nettle --disable-numa --disable-nvmm --disable-opengl --disable-oss --disable-pa --disable-parallels --disable-pipewire --disable-png --disable-qcow1 --disable-qed --disable-qga-vss --disable-rbd --disable-rdma --disable-replication --disable-sdl --disable-sdl-image --disable-seccomp --disable-selinux --disable-smartcard --disable-snappy --disable-sndio --disable-sparse --disable-spice --disable-spice-protocol --disable-stack-protector --disable-tcg --disable-tools --disable-tpm --disable-u2f --disable-usb-redir --disable-vde --disable-vdi --disable-vhdx --disable-vhost-crypto --disable-vhost-kernel --disable-vhost-net --disable-vhost-user --disable-vhost-vdpa --disable-virglrenderer --disable-virtfs --disable-vmdk --disable-vmnet --disable-vnc --disable-vnc-jpeg --disable-vnc-sasl --disable-vpc --disable-vte --disable-vvfat --disable-whpx --disable-xen --disable-xkbcommon --disable-zstd --disable-system --disable-user --disable-linux-user --disable-bsd-user --disable-pie --disable-debug-tcg --disable-werror --disable-alsa --disable-debug-info --enable-tcg --enable-system --enable-gcrypt

  ninja -C build


===========
QEMU README
===========

QEMU is a generic and open source machine & userspace emulator and
virtualizer.

QEMU is capable of emulating a complete machine in software without any
need for hardware virtualization support. By using dynamic translation,
it achieves very good performance. QEMU can also integrate with the Xen
and KVM hypervisors to provide emulated hardware while allowing the
hypervisor to manage the CPU. With hypervisor support, QEMU can achieve
near native performance for CPUs. When QEMU emulates CPUs directly it is
capable of running operating systems made for one machine (e.g. an ARMv7
board) on a different machine (e.g. an x86_64 PC board).

QEMU is also capable of providing userspace API virtualization for Linux
and BSD kernel interfaces. This allows binaries compiled against one
architecture ABI (e.g. the Linux PPC64 ABI) to be run on a host using a
different architecture ABI (e.g. the Linux x86_64 ABI). This does not
involve any hardware emulation, simply CPU and syscall emulation.

QEMU aims to fit into a variety of use cases. It can be invoked directly
by users wishing to have full control over its behaviour and settings.
It also aims to facilitate integration into higher level management
layers, by providing a stable command line interface and monitor API.
It is commonly invoked indirectly via the libvirt library when using
open source applications such as oVirt, OpenStack and virt-manager.

QEMU as a whole is released under the GNU General Public License,
version 2. For full licensing details, consult the LICENSE file.


Documentation
=============

Documentation can be found hosted online at
`<https://www.qemu.org/documentation/>`_. The documentation for the
current development version that is available at
`<https://www.qemu.org/docs/master/>`_ is generated from the ``docs/``
folder in the source tree, and is built by `Sphinx
<https://www.sphinx-doc.org/en/master/>`_.


Building
========

QEMU is multi-platform software intended to be buildable on all modern
Linux platforms, OS-X, Win32 (via the Mingw64 toolchain) and a variety
of other UNIX targets. The simple steps to build QEMU are:


.. code-block:: shell

  mkdir build
  cd build
  ../configure
  make

Additional information can also be found online via the QEMU website:

* `<https://wiki.qemu.org/Hosts/Linux>`_
* `<https://wiki.qemu.org/Hosts/Mac>`_
* `<https://wiki.qemu.org/Hosts/W32>`_


Submitting patches
==================

The QEMU source code is maintained under the GIT version control system.

.. code-block:: shell

   git clone https://gitlab.com/qemu-project/qemu.git

When submitting patches, one common approach is to use 'git
format-patch' and/or 'git send-email' to format & send the mail to the
qemu-devel@nongnu.org mailing list. All patches submitted must contain
a 'Signed-off-by' line from the author. Patches should follow the
guidelines set out in the `style section
<https://www.qemu.org/docs/master/devel/style.html>`_ of
the Developers Guide.

Additional information on submitting patches can be found online via
the QEMU website:

* `<https://wiki.qemu.org/Contribute/SubmitAPatch>`_
* `<https://wiki.qemu.org/Contribute/TrivialPatches>`_

The QEMU website is also maintained under source control.

.. code-block:: shell

  git clone https://gitlab.com/qemu-project/qemu-web.git

* `<https://www.qemu.org/2017/02/04/the-new-qemu-website-is-up/>`_

A 'git-publish' utility was created to make above process less
cumbersome, and is highly recommended for making regular contributions,
or even just for sending consecutive patch series revisions. It also
requires a working 'git send-email' setup, and by default doesn't
automate everything, so you may want to go through the above steps
manually for once.

For installation instructions, please go to:

*  `<https://github.com/stefanha/git-publish>`_

The workflow with 'git-publish' is:

.. code-block:: shell

  $ git checkout master -b my-feature
  $ # work on new commits, add your 'Signed-off-by' lines to each
  $ git publish

Your patch series will be sent and tagged as my-feature-v1 if you need to refer
back to it in the future.

Sending v2:

.. code-block:: shell

  $ git checkout my-feature # same topic branch
  $ # making changes to the commits (using 'git rebase', for example)
  $ git publish

Your patch series will be sent with 'v2' tag in the subject and the git tip
will be tagged as my-feature-v2.

Bug reporting
=============

The QEMU project uses GitLab issues to track bugs. Bugs
found when running code built from QEMU git or upstream released sources
should be reported via:

* `<https://gitlab.com/qemu-project/qemu/-/issues>`_

If using QEMU via an operating system vendor pre-built binary package, it
is preferable to report bugs to the vendor's own bug tracker first. If
the bug is also known to affect latest upstream code, it can also be
reported via GitLab.

For additional information on bug reporting consult:

* `<https://wiki.qemu.org/Contribute/ReportABug>`_


ChangeLog
=========

For version history and release notes, please visit
`<https://wiki.qemu.org/ChangeLog/>`_ or look at the git history for
more detailed information.


Contact
=======

The QEMU community can be contacted in a number of ways, with the two
main methods being email and IRC:

* `<mailto:qemu-devel@nongnu.org>`_
* `<https://lists.nongnu.org/mailman/listinfo/qemu-devel>`_
* #qemu on irc.oftc.net

Information on additional methods of contacting the community can be
found online via the QEMU website:

* `<https://wiki.qemu.org/Contribute/StartHere>`_
