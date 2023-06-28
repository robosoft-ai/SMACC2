#!/bin/bash

# String with the package description obtained from the original machine
#package_description="nvidia-driver-525/unknown,now 525.85.12-0ubuntu1 amd64 [installed]"
#DRIVER=$(apt list --installed | grep nvidia-driver)

echo "Package description: $1"
package_description=$1
#package_description=$DRIVER

regex="([^/]+)/[^[:space:]]+ (.+)"
#regex="([^/]+)/([^[:space:]]+).+"
if [[ $package_description =~ $regex ]]; then
    package_name="${BASH_REMATCH[1]}"
    package_version="${BASH_REMATCH[2]}"
else
    echo "Failed to extract package name and version from the package description."
    exit 1
fi

echo "Package name: $package_name"
echo "Package version: $package_version"

# Check if the package is installed
if dpkg -s "$package_name" >/dev/null 2>&1; then
    echo "Package $package_name is installed."
    echo "Checking if it has version $package_version."

    installed_package_description=$(apt list --installed | grep -w "$package_name")
    echo "Installed package: $installed_package_description"

    if [[ $installed_package_description =~ $regex ]]; then
        installed_version="${BASH_REMATCH[2]}"
    fi
    echo "Installed version: $installed_version"

    # Verify if the installed version matches the desired version
    if [[ "$installed_version" == "$package_version" ]]; then
        echo "Package $package_name-$package_version is already installed."
    else
        echo "WARNING: Package $package_name is installed, but it does not have the same debian package version $package_version."
        DO_INSTALL=1
    fi
else
    echo "Package $package_name is not installed."
    DO_INSTALL=1
fi

if [ "$DO_INSTALL" == "1" ]; then
    echo "Attempting to install package $package_name-$package_version."
    # Attempt to install the package
    sudo apt-get install -y "$package_name"
    
    # Check if the installation was successful
    if dpkg -s "$package_name" >/dev/null 2>&1; then
        echo "Package $package_name-$package_version has been successfully installed."
    else
        echo "Unable to install package $package_name-$package_version."
    fi
else 
    echo "Checking nvidia-driver. Nothing to do. Package $package_name-$package_version is already installed."
fi
