```nix build .#nixosConfigurations.test-pi.config.system.build.sdImage --system aarch64-linux```

```sudo zstd -d result/sd-image/nixos-sd-image-<version-tag-here>-aarch64-linux.img.zst```

```sudo dd bs=4M if=result/sd-image/nixos-sd-image-<version-tag-here>-aarch64-linux.img of=/dev/sda conv=fsync oflag=direct status=progress```

