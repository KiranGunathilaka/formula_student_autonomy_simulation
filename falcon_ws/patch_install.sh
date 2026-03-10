#!/bin/bash
# patch_install.sh
# Run this once after 'colcon build' to fix AMENT_PREFIX_PATH for C++ packages.
# These packages build correctly but colcon overwrites their package.dsv
# without the local_setup.* entries that register them in AMENT_PREFIX_PATH.

INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install"

PACKAGES=(
    falcon_cone_fusion
    falcon_cone_map_builder
    falcon_cone_perception
    falcon_msgs
    falcon_vehicle_comm
    falcon_common
    falcon_localization
)

for pkg in "${PACKAGES[@]}"; do
    pkg_dsv="$INSTALL_DIR/$pkg/share/$pkg/package.dsv"
    if [ ! -f "$pkg_dsv" ]; then
        echo "SKIP $pkg (package.dsv not found)"
        continue
    fi
    if grep -q "local_setup.bash" "$pkg_dsv"; then
        echo "OK   $pkg (already has local_setup entries)"
        continue
    fi
    cat >> "$pkg_dsv" << EOF
source;share/$pkg/local_setup.bash
source;share/$pkg/local_setup.dsv
source;share/$pkg/local_setup.sh
source;share/$pkg/local_setup.zsh
EOF
    echo "PATCHED $pkg"
done

echo ""
echo "Done. Now run: source install/setup.bash"
