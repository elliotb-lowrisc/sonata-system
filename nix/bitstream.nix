# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
{
  pkgs,
  pythonEnv,
  sonata-system-software,
  sonataGatewareFiles,
  FLAKE_GIT_COMMIT,
  FLAKE_GIT_DIRTY,
}: let
  inherit (pkgs.lib) fileset;

  sonataFpgaFileset = fileset.toSource {
    root = ../.;
    fileset = fileset.unions [
      sonataGatewareFiles
      (fileset.fileFilter (file: file.hasExt "xdc") ../data)
      ../flow
    ];
  };
in {
  build = pkgs.writeShellApplication {
    name = "bitstream-build";
    runtimeInputs = [pythonEnv];
    runtimeEnv = {inherit FLAKE_GIT_COMMIT FLAKE_GIT_DIRTY;};
    text = ''
      fusesoc --cores-root=${sonataFpgaFileset} \
        run --target=synth --build lowrisc:sonata:system \
        --SRAMInitFile=${sonata-system-software}/share/boot_loader.vmem
    '';
  };

  load = pkgs.writeShellApplication {
    name = "bitstream-load";
    runtimeInputs = [pythonEnv pkgs.openfpgaloader];
    text = ''
      BITSTREAM=$(find ./ -type f -name "lowrisc_sonata_system_0.bit")
      openFPGALoader -c ft4232 "$BITSTREAM"
    '';
  };
}