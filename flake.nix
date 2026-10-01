{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-26.05";
  };

  outputs = {
    self,
    nixpkgs,
  }: let
    supportedSystems = [
      "x86_64-linux"
      "aarch64-linux"
      "aarch64-darwin"
    ];
    forAllSystems = nixpkgs.lib.genAttrs supportedSystems;
  in {
    devShells = forAllSystems (system: let
      pkgs = nixpkgs.legacyPackages.${system};
    in {
      default = pkgs.mkShell {
        buildInputs = with pkgs; [
          git
          pkg-config
          cmake
          python3
          gcc-arm-embedded
          picotool
          # Keep the validated lint/format baseline independent of channel bumps.
          # LLVM 21 adds diagnostics that need a separate source-baseline migration.
          llvmPackages_19.clang
          llvmPackages_19.clang-tools
          picocom
          pre-commit
        ];
        # CMake fetches the same pinned SDK as CI; nixpkgs' SDK can lag behind.
        # Do not export PICO_SDK_PATH and silently bypass pico_sdk_import.cmake.
        # CMake also fetches SDK-matched build-time UF2 tooling.
      };
    });
  };
}
